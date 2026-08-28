#include "wifi_telemetry.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "acoustic_protocol.h"
#include "app_config.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "usb_link.h"

#define WIFI_TELEMETRY_CONNECTED_BIT          (1U << 0)
#define WIFI_TELEMETRY_JSON_CAPACITY          (1024U)

static EventGroupHandle_t s_wifi_event_group;
static struct sockaddr_in s_destination;
static uint32_t s_wifi_reconnect_count;
static uint32_t s_udp_send_count;
static uint32_t s_udp_error_count;

static uint32_t wifi_telemetry_uptime_ms(void) {
    return (uint32_t) ((uint64_t) esp_timer_get_time() / 1000ULL);
}

static int wifi_telemetry_flag(uint8_t flags, uint8_t mask) {
    return ((flags & mask) != 0U) ? 1 : 0;
}

static char const *wifi_telemetry_think_state(uint8_t state) {
    static char const *const names[] = {
        "WAIT_LINK",
        "LISTEN",
        "STEER_PREP",
        "MOVE_STEP",
        "SETTLE",
        "COOLDOWN",
        "FAULT",
    };
    return (state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

static void wifi_telemetry_event_handler(void *argument,
                                         esp_event_base_t event_base,
                                         int32_t event_id,
                                         void *event_data) {
    (void) argument;
    (void) event_data;

    if ((event_base == WIFI_EVENT) &&
        (event_id == WIFI_EVENT_STA_START)) {
        (void) esp_wifi_connect();
    } else if ((event_base == WIFI_EVENT) &&
               (event_id == WIFI_EVENT_STA_DISCONNECTED)) {
        xEventGroupClearBits(s_wifi_event_group,
                             WIFI_TELEMETRY_CONNECTED_BIT);
        s_wifi_reconnect_count++;
        (void) esp_wifi_connect();
    } else if ((event_base == IP_EVENT) &&
               (event_id == IP_EVENT_STA_GOT_IP)) {
        xEventGroupSetBits(s_wifi_event_group,
                           WIFI_TELEMETRY_CONNECTED_BIT);
    }
}

static esp_err_t wifi_telemetry_station_start(void) {
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        return err;
    }
    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_init);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_register(WIFI_EVENT,
                                     ESP_EVENT_ANY_ID,
                                     wifi_telemetry_event_handler,
                                     NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_register(IP_EVENT,
                                     IP_EVENT_STA_GOT_IP,
                                     wifi_telemetry_event_handler,
                                     NULL);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t wifi_config = {0};
    (void) snprintf((char *) wifi_config.sta.ssid,
                    sizeof(wifi_config.sta.ssid),
                    "%s",
                    APP_WIFI_SSID);
    (void) snprintf((char *) wifi_config.sta.password,
                    sizeof(wifi_config.sta.password),
                    "%s",
                    APP_WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    return err;
}

static int wifi_telemetry_format_json(
    char *json,
    size_t capacity,
    acoustic_rover_telemetry_t const *telemetry,
    acoustic_frame_t const *frame,
    uint32_t received_at_ms,
    int rssi_dbm) {
    uint32_t const now_ms = wifi_telemetry_uptime_ms();
    uint8_t const flags = telemetry->flags;

    return snprintf(
        json,
        capacity,
        "{\"schema\":1,\"esp_ms\":%lu,\"cpu_valid\":true,"
        "\"cpu_age_ms\":%lu,\"wifi\":{\"connected\":1,"
        "\"rssi_dbm\":%d,\"reconnects\":%lu},"
        "\"udp\":{\"sent\":%lu,\"errors\":%lu},"
        "\"usb\":{\"mounted\":%d,\"rx_drops\":%lu,"
        "\"cpu_ms\":%lu,\"cpu_seq\":%lu,\"state\":%u,"
        "\"configured\":%d,\"hello\":%d},"
        "\"audio\":{\"observation\":%d,\"sequence\":%lu,"
        "\"age_ms\":%lu,\"doa_deg\":%u,"
        "\"level_dbfs_x100\":%d,\"peak_dbfs_x100\":%d,"
        "\"vad\":%u,\"xvf_status\":%u,\"xvf_raw_status\":%u,"
        "\"doa_fallback\":%d,\"flags\":%u,"
        "\"pcm_frames\":%lu,\"crc_errors\":%lu},"
        "\"think\":{\"state\":%u,\"name\":\"%s\","
        "\"link_ready\":%d,\"new_observation\":%d,"
        "\"faults\":%lu,\"steering_deg\":%d},"
        "\"command\":{\"left_rpm\":%d,\"right_rpm\":%d,"
        "\"servo_deg\":[%d,%d,%d,%d],\"enable\":%d,"
        "\"emergency_stop\":%d,\"stale\":%d,"
        "\"target_age_ms\":%lu,\"sequence\":%lu,"
        "\"sent\":%lu,\"last_error\":%ld}}\n",
        (unsigned long) now_ms,
        (unsigned long) (now_ms - received_at_ms),
        rssi_dbm,
        (unsigned long) s_wifi_reconnect_count,
        (unsigned long) s_udp_send_count,
        (unsigned long) s_udp_error_count,
        usb_link_is_mounted() ? 1 : 0,
        (unsigned long) usb_link_rx_drop_count(),
        (unsigned long) frame->uptime_ms,
        (unsigned long) frame->sequence,
        (unsigned int) telemetry->usb_state,
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_USB_CONFIGURED),
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_HELLO_RECEIVED),
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_OBSERVATION),
        (unsigned long) telemetry->observation_sequence,
        (unsigned long) telemetry->observation_age_ms,
        (unsigned int) telemetry->doa_deg,
        telemetry->level_dbfs_x100,
        telemetry->peak_dbfs_x100,
        (unsigned int) telemetry->vad,
        (unsigned int) telemetry->xvf_status,
        (unsigned int) telemetry->xvf_raw_status,
        wifi_telemetry_flag(telemetry->audio_flags,
            ACOUSTIC_AUDIO_FLAG_DOA_FALLBACK),
        (unsigned int) telemetry->audio_flags,
        (unsigned long) telemetry->audio_frame_count,
        (unsigned long) telemetry->audio_crc_error_count,
        (unsigned int) telemetry->think_state,
        wifi_telemetry_think_state(telemetry->think_state),
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_LINK_READY),
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_NEW_OBSERVATION),
        (unsigned long) telemetry->fault_flags,
        telemetry->steering_deg,
        telemetry->left_target_rpm,
        telemetry->right_target_rpm,
        telemetry->servo_target_deg[0],
        telemetry->servo_target_deg[1],
        telemetry->servo_target_deg[2],
        telemetry->servo_target_deg[3],
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_ACTUATOR_ENABLE),
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_EMERGENCY_STOP),
        wifi_telemetry_flag(flags,
            ACOUSTIC_TELEMETRY_FLAG_COMMAND_STALE),
        (unsigned long) telemetry->command_target_age_ms,
        (unsigned long) telemetry->command_sequence,
        (unsigned long) telemetry->command_send_count,
        (long) telemetry->command_last_error);
}

static int wifi_telemetry_format_heartbeat(char *json,
                                           size_t capacity,
                                           int rssi_dbm) {
    return snprintf(
        json,
        capacity,
        "{\"schema\":1,\"esp_ms\":%lu,\"cpu_valid\":false,"
        "\"wifi\":{\"connected\":1,\"rssi_dbm\":%d,"
        "\"reconnects\":%lu},\"udp\":{\"sent\":%lu,"
        "\"errors\":%lu},\"usb\":{\"mounted\":%d,"
        "\"rx_drops\":%lu}}\n",
        (unsigned long) wifi_telemetry_uptime_ms(),
        rssi_dbm,
        (unsigned long) s_wifi_reconnect_count,
        (unsigned long) s_udp_send_count,
        (unsigned long) s_udp_error_count,
        usb_link_is_mounted() ? 1 : 0,
        (unsigned long) usb_link_rx_drop_count());
}

static void wifi_telemetry_task(void *argument) {
    (void) argument;

    acoustic_protocol_parser_t parser;
    acoustic_rover_telemetry_t latest_telemetry = {0};
    acoustic_frame_t latest_frame = {0};
    uint32_t received_at_ms = 0U;
    uint32_t last_send_ms = 0U;
    bool telemetry_valid = false;
    int socket_fd = -1;
    uint8_t rx_data[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    acoustic_protocol_parser_init(&parser);

    while (true) {
        size_t received_length = 0U;
        esp_err_t const rx_err = usb_link_receive(
            rx_data,
            sizeof(rx_data),
            &received_length,
            APP_TELEMETRY_USB_POLL_MS);
        if (rx_err == ESP_OK) {
            for (size_t index = 0U; index < received_length; index++) {
                acoustic_frame_t frame;
                acoustic_parse_result_t const parse_result =
                    acoustic_protocol_parser_push(&parser,
                                                  rx_data[index],
                                                  &frame);
                if ((parse_result == ACOUSTIC_PARSE_FRAME_READY) &&
                    acoustic_protocol_decode_rover_telemetry(
                        &frame,
                        &latest_telemetry)) {
                    latest_frame = frame;
                    received_at_ms = wifi_telemetry_uptime_ms();
                    telemetry_valid = true;
                }
            }
        }

        uint32_t const now_ms = wifi_telemetry_uptime_ms();
        bool const connected = wifi_telemetry_is_connected();
        if (!connected) {
            if (socket_fd >= 0) {
                close(socket_fd);
                socket_fd = -1;
            }
            continue;
        }
        if ((now_ms - last_send_ms) < APP_UDP_TELEMETRY_PERIOD_MS) {
            continue;
        }
        last_send_ms = now_ms;

        if (socket_fd < 0) {
            socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (socket_fd < 0) {
                s_udp_error_count++;
                continue;
            }
        }

        wifi_ap_record_t access_point = {0};
        int rssi_dbm = -127;
        if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK) {
            rssi_dbm = access_point.rssi;
        }

        char json[WIFI_TELEMETRY_JSON_CAPACITY];
        int const json_length = telemetry_valid ?
            wifi_telemetry_format_json(json,
                                       sizeof(json),
                                       &latest_telemetry,
                                       &latest_frame,
                                       received_at_ms,
                                       rssi_dbm) :
            wifi_telemetry_format_heartbeat(json,
                                            sizeof(json),
                                            rssi_dbm);
        if ((json_length <= 0) ||
            ((size_t) json_length >= sizeof(json))) {
            s_udp_error_count++;
            continue;
        }

        int const sent = sendto(socket_fd,
                                json,
                                (size_t) json_length,
                                0,
                                (struct sockaddr *) &s_destination,
                                sizeof(s_destination));
        if (sent == json_length) {
            s_udp_send_count++;
        } else {
            s_udp_error_count++;
        }
    }
}

bool wifi_telemetry_is_connected(void) {
    if (s_wifi_event_group == NULL) {
        return false;
    }
    return (xEventGroupGetBits(s_wifi_event_group) &
            WIFI_TELEMETRY_CONNECTED_BIT) != 0U;
}

esp_err_t wifi_telemetry_start(void) {
    if ((APP_WIFI_SSID[0] == '\0') ||
        (APP_UDP_DESTINATION_IPV4[0] == '\0') ||
        (strlen(APP_WIFI_SSID) >= sizeof(((wifi_config_t *) 0)->sta.ssid)) ||
        (strlen(APP_WIFI_PASSWORD) >=
         sizeof(((wifi_config_t *) 0)->sta.password))) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_destination, 0, sizeof(s_destination));
    s_destination.sin_family = AF_INET;
    s_destination.sin_port = htons(APP_UDP_DESTINATION_PORT);
    if (inet_pton(AF_INET,
                  APP_UDP_DESTINATION_IPV4,
                  &s_destination.sin_addr) != 1) {
        return ESP_ERR_INVALID_ARG;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t const station_err = wifi_telemetry_station_start();
    if (station_err != ESP_OK) {
        return station_err;
    }

    BaseType_t const task_result = xTaskCreate(
        wifi_telemetry_task,
        "wifi_telemetry",
        APP_TELEMETRY_TASK_STACK_SIZE,
        NULL,
        APP_TELEMETRY_TASK_PRIORITY,
        NULL);
    return (task_result == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
