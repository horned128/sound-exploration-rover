/** =================================================================*
 * @file   wifi_telemetry.c
 * @brief  Wi-Fiテレメトリー送信
 * ================================================================= */
#include "wifi_telemetry.h"                                 /* Wi-Fiテレメトリー送信API */

#include "acoustic_protocol.h"                              /* CPU0テレメトリー通信プロトコル */
#include "app_config.h"                                     /* Wi-FiとUDP送信設定 */
#include "audio_capture.h"                                  /* ESP32音声DSP診断値 */
#include "esp_event.h"                                      /* ESP-IDFイベントAPI */
#include "esp_netif.h"                                      /* ESP-IDFネットワークIF API */
#include "esp_timer.h"                                      /* 単調時刻取得API */
#include "esp_wifi.h"                                       /* ESP-IDF Wi-Fi API */
#include "freertos/FreeRTOS.h"                              /* FreeRTOS基本型 */
#include "freertos/event_groups.h"                          /* FreeRTOSイベントグループAPI */
#include "freertos/task.h"                                  /* FreeRTOSタスクAPI */
#include "lwip/inet.h"                                      /* IPv4文字列変換API */
#include "lwip/sockets.h"                                   /* UDPソケットAPI */
#include "nvs_flash.h"                                      /* Wi-Fi永続設定領域API */
#include "usb_link.h"                                       /* CPU0からのUSB CDC受信API */
#include <stdbool.h>                                        /* 真偽値 */
#include <stdint.h>                                         /* 固定幅整数型 */
#include <stdio.h>                                          /* JSON整形API */
#include <string.h>                                         /* 文字列・メモリー操作API */

#define WIFI_TELEMETRY_CONNECTED_BIT       (1U << 0)
#define WIFI_TELEMETRY_JSON_CAPACITY       (3072U)
#define WIFI_DIAGNOSTIC_QUEUE_CAPACITY      (8U)
#define WIFI_DIAGNOSTIC_JSON_CAPACITY       (2048U)

typedef enum {
    WIFI_DIAGNOSTIC_SUMMARY = 0,
    WIFI_DIAGNOSTIC_SAMPLE,
} wifi_diagnostic_kind_t;

typedef struct {
    wifi_diagnostic_kind_t kind;
    uint32_t generation;
    uint8_t sample_index;
    uint8_t sample_count;
    uint8_t cpu_drop_count;
    bool snapshot_valid;
    acoustic_ai_lab_snapshot_t snapshot;
    int8_t vector[ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE * 3U];
} wifi_diagnostic_event_t;

typedef struct {
    bool active;
    uint32_t generation;
    uint8_t received_mask;
    uint8_t cpu_drop_count;
    int8_t vector[ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE * 3U];
} wifi_summary_assembly_t;

typedef struct {
    bool active;
    uint32_t generation;
    uint8_t sample_index;
    uint8_t sample_count;
    uint8_t received_mask;
    int8_t vector[ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE * 3U];
} wifi_profile_assembly_t;

static EventGroupHandle_t s_wifi_event_group;               /**< Wi-Fi接続状態イベント */
static struct sockaddr_in s_destination;                    /**< UDP送信先IPv4アドレス */
static uint32_t s_wifi_reconnect_count;                     /**< Wi-Fi再接続回数 */
static uint32_t s_udp_send_count;                           /**< UDP送信成功回数 */
static uint32_t s_udp_error_count;                          /**< UDP送信失敗回数 */
static wifi_diagnostic_event_t s_diagnostic_queue[WIFI_DIAGNOSTIC_QUEUE_CAPACITY]; /**< bounded event queue */
static uint8_t s_diagnostic_queue_head;
static uint8_t s_diagnostic_queue_count;
static uint32_t s_diagnostic_drop_count;
static wifi_summary_assembly_t s_summary_assembly;
static uint32_t s_summary_seen_generation;
static bool s_summary_seen_generation_valid;
static wifi_profile_assembly_t s_profile_assembly;
static uint32_t s_profile_seen_generation;
static uint8_t s_profile_seen_mask;
static bool s_profile_seen_generation_valid;

static uint32_t wifi_telemetry_uptime_ms(void);             /* 起動からの経過時刻取得 */
static int wifi_telemetry_flag(uint8_t flags, uint8_t mask);/* フラグをJSON真偽値へ変換 */
static char const * wifi_telemetry_think_state(uint8_t state); /* 思考状態名取得 */
static char const * wifi_telemetry_autonomy_mode(uint8_t mode); /* 自律モード名取得 */
static char const * wifi_telemetry_sensor_rule(uint8_t rule); /* センサー走行ルール名取得 */
static char const * wifi_telemetry_infer_status(uint8_t status); /* 音響認識判定名取得 */
/* ESP-IDF Wi-Fi/IPイベント処理 */
static void wifi_telemetry_event_handler(void * argument, esp_event_base_t event_base, int32_t event_id,
                                         void * event_data);
static esp_err_t wifi_telemetry_station_start(void);        /* Wi-Fiステーション開始 */
/* CPU0テレメトリーのJSON整形 */
static int wifi_telemetry_format_json(char * json, size_t capacity, acoustic_rover_telemetry_t const * telemetry,
                                      acoustic_frame_t const * frame,
                                      acoustic_actuator_telemetry_t const * actuator_telemetry,
                                      bool actuator_frame_valid,
                                      acoustic_pose_telemetry_t const * pose_telemetry,
                                      bool pose_frame_valid, uint32_t received_at_ms,
                                      uint32_t actuator_received_at_ms, uint32_t pose_received_at_ms,
                                      acoustic_nav_diagnostics_t const * nav_diagnostics,
                                      bool nav_frame_valid, uint32_t nav_received_at_ms,
                                      int rssi_dbm, audio_capture_snapshot_t const * esp_audio,
                                      uint32_t feature_fps_x100);
/* 接続状態JSON整形 */
static int wifi_telemetry_format_heartbeat(char * json, size_t capacity, int rssi_dbm,
                                           audio_capture_snapshot_t const * esp_audio,
                                           uint32_t feature_fps_x100);
static bool wifi_telemetry_summary_chunk_accept(acoustic_ai_lab_summary_chunk_t const * chunk,
                                                acoustic_ai_lab_snapshot_t const * snapshot,
                                                bool snapshot_valid);
static bool wifi_telemetry_profile_chunk_accept(acoustic_ai_lab_profile_chunk_t const * chunk);
static bool wifi_telemetry_diagnostic_enqueue(wifi_diagnostic_event_t const * event);
static bool wifi_telemetry_diagnostic_send(int socket_fd);
static void wifi_telemetry_task(void * argument);           /* USB受信・UDP送信タスク */

/** =================================================================*
 * @brief  起動経過時刻取得
 * @return 起動からの経過時刻[ms]
 * ================================================================= */
static uint32_t wifi_telemetry_uptime_ms(void) {
    return (uint32_t) ((uint64_t) esp_timer_get_time() / 1000ULL);
}

#if APP_AUDIO_DATASET_STREAM_ENABLE
/* 256 sample PCM16を1チャネルずつ送る。既存ch0 JSONとの互換性を保つ。 */
static void wifi_telemetry_pcm_channel_send(int socket_fd, audio_capture_pcm_block_t const * block,
                                            unsigned int channel) {
    static char json[1024];
    static char const alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int length = snprintf(json, sizeof(json),
                          "{\"record_type\":\"%s\",\"schema\":1,\"channel\":%u,\"esp_ms\":%lu,"
                          "\"first_sample\":%lu,\"sample_count\":%u,\"dropped_blocks\":%lu,\"pcm16le_b64\":\"",
                          (0U == channel) ? "acoustic_pcm" : "acoustic_pcm_ch1", channel,
                          (unsigned long) wifi_telemetry_uptime_ms(),
                          (unsigned long) block->first_sample, (unsigned int) block->sample_count,
                          (unsigned long) block->dropped_blocks);
    if ((length < 0) || ((size_t) length >= sizeof(json))) {
        s_udp_error_count++;
        return;
    }
    size_t const byte_count = (size_t) block->sample_count * 2U;
    for (size_t offset = 0U; offset < byte_count; offset += 3U) {
        uint32_t bits = 0U;
        for (size_t part = 0U; part < 3U; part++) {
            size_t const byte_index = offset + part;
            if (byte_index < byte_count) {
#if APP_AUDIO_STEREO_DIAGNOSTIC_ENABLE
                uint16_t const sample = (uint16_t) ((0U == channel) ? block->samples[byte_index / 2U] :
                                                              block->second_channel[byte_index / 2U]);
#else
                uint16_t const sample = (uint16_t) block->samples[byte_index / 2U];
#endif
                uint32_t const value = (byte_index & 1U) ? (sample >> 8U) : (sample & 0xFFU);
                bits |= value << (16U - (part * 8U));
            }
        }
        if (((size_t) length + 7U) >= sizeof(json)) {
            length = -1;
            break;
        }
        json[length++] = alphabet[(bits >> 18U) & 63U];
        json[length++] = alphabet[(bits >> 12U) & 63U];
        json[length++] = ((offset + 1U) < byte_count) ? alphabet[(bits >> 6U) & 63U] : '=';
        json[length++] = ((offset + 2U) < byte_count) ? alphabet[bits & 63U] : '=';
    }
    if (length < 0) {
        s_udp_error_count++;
        return;
    }
    json[length++] = '"';
    json[length++] = '}';
    json[length++] = '\n';
    if (sendto(socket_fd, json, (size_t) length, 0,
               (struct sockaddr *) &s_destination, sizeof(s_destination)) == length) {
        s_udp_send_count++;
    } else {
        s_udp_error_count++;
    }
}

/* PCMブロック256 sample = 16 ms。最大2件/20 ms poll。 */
static void wifi_telemetry_pcm_send(int socket_fd) {
    for (unsigned int packet = 0U; packet < 2U; packet++) {
        audio_capture_pcm_block_t block;
        if (!audio_capture_pcm_block_take(&block)) {
            break;
        }
        wifi_telemetry_pcm_channel_send(socket_fd, &block, 0U);
#if APP_AUDIO_STEREO_DIAGNOSTIC_ENABLE
        wifi_telemetry_pcm_channel_send(socket_fd, &block, 1U);
#endif
    }
}
#endif

/** =================================================================*
 * @brief  JSON真偽値変換
 * @param[in] flags フラグ集合
 * @param[in] mask 判定対象ビット
 * @return 対象ビットが設定済みなら1、未設定なら0
 * ================================================================= */
static int wifi_telemetry_flag(uint8_t flags, uint8_t mask) {
    return ((flags & mask) != 0U) ? 1 : 0;
}

/** =================================================================*
 * @brief  思考状態名取得
 * @param[in] state CPU0思考状態値
 * @return JSONに記録する状態名
 * ================================================================= */
static char const * wifi_telemetry_think_state(uint8_t state) {
    static char const * const names[] = {
        "WAIT_LINK", "LISTEN", "STEER_PREP", "MOVE_STEP", "SETTLE", "COOLDOWN", "SENSOR_SAFE_STOP",
        "SENSOR_FORWARD", "SENSOR_CAUTION_FORWARD", "SENSOR_TURN_LEFT", "SENSOR_TURN_RIGHT",
        "SENSOR_BLOCKED_STOP", "SENSOR_IMU_STOP", "FAULT",
        "SENSOR_PIVOT_LEFT", "SENSOR_PIVOT_RIGHT", "SENSOR_BACKUP",
        "SPIN_PREP", "SPIN_STEP", "SPIN_NO_PROGRESS", "ARRIVAL_VERIFY", "ARRIVED", "WAIT_RESTART",
    };
    return (state < (sizeof(names) / sizeof(names[0]))) ? names[state] : "UNKNOWN";
}

/** =================================================================*
 * @brief  自律モード名を取得
 * @param[in] mode CPU0の自律モード値
 * @return JSONに記録する自律モード名
 * ================================================================= */
static char const * wifi_telemetry_autonomy_mode(uint8_t mode) {
    return (0U == mode) ? "SOUND_FOLLOW" : (1U == mode) ? "SENSOR_RULE" : "UNKNOWN";
}

/** =================================================================*
 * @brief  センサー走行ルール名を取得
 * @param[in] rule CPU0が選択したセンサー走行ルール値
 * @return JSONに記録するルール名
 * ================================================================= */
static char const * wifi_telemetry_sensor_rule(uint8_t rule) {
    static char const * const names[] = {
        "SAFE_STOP", "FORWARD", "CAUTION_FORWARD", "TURN_LEFT", "TURN_RIGHT", "BLOCKED_STOP", "IMU_STOP",
        "PIVOT_LEFT", "PIVOT_RIGHT", "BACKUP",
    };
    return (rule < (sizeof(names) / sizeof(names[0]))) ? names[rule] : "UNKNOWN";
}

/** =================================================================*
 * @brief  音響認識判定名を取得
 * @param[in] status 音響認識判定値
 * @return JSONに記録する判定名
 * ================================================================= */
static char const * wifi_telemetry_infer_status(uint8_t status) {
    static char const * const names[] = {
        "INVALID", "INDETERMINATE", "NOT_READY", "NOT_TARGET", "TARGET",
    };
    return (status < (sizeof(names) / sizeof(names[0]))) ? names[status] : "UNKNOWN";
}

/** =================================================================*
 * @brief  音響推論器名を取得
 * @param[in] kind 推論器種別値
 * @return JSONに記録する分類器名
 * ================================================================= */
static char const * wifi_telemetry_classifier_name(uint8_t kind) {
    switch (kind) {
        case ACOUSTIC_INFER_CLASSIFIER_DSP_SUMMARY:
            return "DSP";
        case ACOUSTIC_INFER_CLASSIFIER_NN_EMBEDDING:
            return "TFLM";
        case ACOUSTIC_INFER_CLASSIFIER_NONE:
        default:
            return "NONE";
    }
}

/** =================================================================*
 * @brief 低優先度診断イベントを有限UDP queueへ追加
 * @param[in] event 完成した特徴量または保存見本イベント
 * @return queueへ追加した場合true
 * ================================================================= */
static bool wifi_telemetry_diagnostic_enqueue(wifi_diagnostic_event_t const * event) {
    if (event == NULL) {
        return false;
    }
    if (s_diagnostic_queue_count >= WIFI_DIAGNOSTIC_QUEUE_CAPACITY) {
        s_diagnostic_drop_count++;
        return false;
    }
    uint8_t const tail = (uint8_t) ((s_diagnostic_queue_head + s_diagnostic_queue_count) %
                                    WIFI_DIAGNOSTIC_QUEUE_CAPACITY);
    s_diagnostic_queue[tail] = *event;
    s_diagnostic_queue_count++;
    return true;
}

/** =================================================================*
 * @brief 3つの64-byte要約chunkを世代単位で組み立て
 * @param[in] chunk 受信chunk
 * @param[in] snapshot 最新snapshot
 * @param[in] snapshot_valid snapshot受信済み
 * @return 完成した要約イベントを処理した場合true
 * ================================================================= */
static bool wifi_telemetry_summary_chunk_accept(acoustic_ai_lab_summary_chunk_t const * chunk,
                                                acoustic_ai_lab_snapshot_t const * snapshot,
                                                bool snapshot_valid) {
    if ((chunk == NULL) || (chunk->chunk_count != 3U) || (chunk->chunk_index >= 3U) ||
        (chunk->schema_version != 1U)) {
        s_diagnostic_drop_count++;
        return false;
    }
    if (!s_summary_assembly.active && s_summary_seen_generation_valid &&
        ((int32_t) (chunk->feature_generation - s_summary_seen_generation) <= 0)) {
        return false;
    }
    if (s_summary_assembly.active && (s_summary_assembly.generation != chunk->feature_generation)) {
        if ((int32_t) (chunk->feature_generation - s_summary_assembly.generation) <= 0) {
            s_diagnostic_drop_count++;
            return false;
        }
        if (s_summary_assembly.active && (s_summary_assembly.received_mask != 0x07U)) {
            s_diagnostic_drop_count++;
        }
        memset(&s_summary_assembly, 0, sizeof(s_summary_assembly));
        s_summary_assembly.active = true;
        s_summary_assembly.generation = chunk->feature_generation;
    } else if (!s_summary_assembly.active) {
        memset(&s_summary_assembly, 0, sizeof(s_summary_assembly));
        s_summary_assembly.active = true;
        s_summary_assembly.generation = chunk->feature_generation;
    }

    uint8_t const bit = (uint8_t) (1U << chunk->chunk_index);
    if ((s_summary_assembly.received_mask & bit) != 0U) {
        return false;
    }
    memcpy(&s_summary_assembly.vector[(size_t) chunk->chunk_index * ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE],
           chunk->data, ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE);
    s_summary_assembly.received_mask |= bit;
    s_summary_assembly.cpu_drop_count = chunk->cpu_drop_count;
    if (s_summary_assembly.received_mask != 0x07U) {
        return false;
    }

    wifi_diagnostic_event_t event = {0};
    event.kind = WIFI_DIAGNOSTIC_SUMMARY;
    event.generation = s_summary_assembly.generation;
    event.cpu_drop_count = s_summary_assembly.cpu_drop_count;
    event.snapshot_valid = snapshot_valid && (snapshot != NULL) && (snapshot->schema_version >= 2U) &&
                           (snapshot->feature_generation == event.generation);
    if (event.snapshot_valid) {
        event.snapshot = *snapshot;
    }
    memcpy(event.vector, s_summary_assembly.vector, sizeof(event.vector));
    (void) wifi_telemetry_diagnostic_enqueue(&event);
    s_summary_seen_generation = event.generation;
    s_summary_seen_generation_valid = true;
    s_summary_assembly.active = false;
    return true;
}

/** =================================================================*
 * @brief 保存見本の3 chunkを組み立て、sampleごとの診断イベントをqueue
 * @param[in] chunk 受信保存見本chunk
 * @return 完成した見本を処理した場合true
 * ================================================================= */
static bool wifi_telemetry_profile_chunk_accept(acoustic_ai_lab_profile_chunk_t const * chunk) {
    if ((chunk == NULL) || (chunk->chunk_count != 3U) || (chunk->chunk_index >= 3U) ||
        (chunk->sample_count == 0U) || (chunk->sample_count > 5U) ||
        (chunk->sample_index >= chunk->sample_count)) {
        s_diagnostic_drop_count++;
        return false;
    }
    if (s_profile_seen_generation_valid && (s_profile_seen_generation != chunk->profile_generation) &&
        ((int32_t) (chunk->profile_generation - s_profile_seen_generation) <= 0)) {
        return false;
    }
    if (!s_profile_seen_generation_valid || (s_profile_seen_generation != chunk->profile_generation)) {
        s_profile_seen_generation = chunk->profile_generation;
        s_profile_seen_mask = 0U;
        s_profile_seen_generation_valid = true;
        s_profile_assembly.active = false;
    }
    uint8_t const sample_bit = (uint8_t) (1U << chunk->sample_index);
    if ((s_profile_seen_mask & sample_bit) != 0U) {
        return false;
    }
    if (!s_profile_assembly.active || (s_profile_assembly.generation != chunk->profile_generation) ||
        (s_profile_assembly.sample_index != chunk->sample_index)) {
        if (s_profile_assembly.active && (s_profile_assembly.received_mask != 0x07U)) {
            s_diagnostic_drop_count++;
        }
        memset(&s_profile_assembly, 0, sizeof(s_profile_assembly));
        s_profile_assembly.active = true;
        s_profile_assembly.generation = chunk->profile_generation;
        s_profile_assembly.sample_index = chunk->sample_index;
        s_profile_assembly.sample_count = chunk->sample_count;
    }

    uint8_t const bit = (uint8_t) (1U << chunk->chunk_index);
    if ((s_profile_assembly.received_mask & bit) != 0U) {
        return false;
    }
    memcpy(&s_profile_assembly.vector[(size_t) chunk->chunk_index * ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE],
           chunk->data, ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE);
    s_profile_assembly.received_mask |= bit;
    if (s_profile_assembly.received_mask != 0x07U) {
        return false;
    }

    wifi_diagnostic_event_t event = {0};
    event.kind = WIFI_DIAGNOSTIC_SAMPLE;
    event.generation = s_profile_assembly.generation;
    event.sample_index = s_profile_assembly.sample_index;
    event.sample_count = s_profile_assembly.sample_count;
    memcpy(event.vector, s_profile_assembly.vector, sizeof(event.vector));
    (void) wifi_telemetry_diagnostic_enqueue(&event);
    s_profile_seen_mask |= sample_bit;
    s_profile_assembly.active = false;
    return true;
}

/** =================================================================*
 * @brief 診断イベントを通常状態JSONとは別のUDP datagramで送信
 * @param[in] socket_fd 送信socket
 * @return イベント送信が成功した場合true
 * ================================================================= */
static bool wifi_telemetry_diagnostic_send(int socket_fd) {
    if ((socket_fd < 0) || (s_diagnostic_queue_count == 0U)) {
        return false;
    }

    wifi_diagnostic_event_t const * const event = &s_diagnostic_queue[s_diagnostic_queue_head];
    static char json[WIFI_DIAGNOSTIC_JSON_CAPACITY];
    int length = 0;
    if (event->kind == WIFI_DIAGNOSTIC_SUMMARY) {
        acoustic_ai_lab_snapshot_t const * snapshot = &event->snapshot;
        bool const valid = event->snapshot_valid;
        uint8_t const state = valid ? (uint8_t) (snapshot->reserved[0] & ACOUSTIC_AI_LAB_MATCH_STATE_MASK) :
                               ACOUSTIC_AI_LAB_MATCH_UNKNOWN;
        uint8_t const strong_count = valid ? (uint8_t) ((snapshot->reserved[0] &
            ACOUSTIC_AI_LAB_MATCH_STRONG_COUNT_MASK) >> ACOUSTIC_AI_LAB_MATCH_STRONG_COUNT_SHIFT) : 0U;
        uint16_t const age_ticks = valid ? (uint16_t) ((uint16_t) snapshot->reserved[1] |
                                                       ((uint16_t) snapshot->reserved[2] << 8U)) :
                                                ACOUSTIC_AI_LAB_MATCH_AGE_INVALID;
        uint32_t const age_ms = (age_ticks == ACOUSTIC_AI_LAB_MATCH_AGE_INVALID) ? UINT32_MAX :
                                (uint32_t) age_ticks * 10U;
        char const * match_name = (state == ACOUSTIC_AI_LAB_MATCH_CONFIRMED) ? "CONFIRMED" :
                                  (state == ACOUSTIC_AI_LAB_MATCH_UNCERTAIN) ? "UNCERTAIN" :
                                  (state == ACOUSTIC_AI_LAB_MATCH_EXPIRED) ? "EXPIRED" : "UNKNOWN";
        char const * reason = !valid ? "snapshot_unavailable" :
                              (state == ACOUSTIC_AI_LAB_MATCH_UNKNOWN) ? "baseline_policy" :
                              (state == ACOUSTIC_AI_LAB_MATCH_UNCERTAIN) ?
                                  ((strong_count > 0U) ? "strong_mismatch_grace" : "not_target_grace") :
                              (strong_count >= 2U) ? "two_strong_mismatches" :
                              ((age_ticks != ACOUSTIC_AI_LAB_MATCH_AGE_INVALID) && (age_ms >= 2500U)) ?
                                  "target_timeout" :
                              (snapshot->infer_status == 4U) ? "target" :
                              (age_ticks == ACOUSTIC_AI_LAB_MATCH_AGE_INVALID) ? "target_not_seen" : "target_expired";
        float const distance = (valid && (snapshot->cosine_distance_x1000 != UINT16_MAX)) ?
                               ((float) snapshot->cosine_distance_x1000 / 1000.0f) : -1.0f;
        float const threshold = valid ? ((float) snapshot->identifier_threshold_x1000 / 1000.0f) : 0.0f;
        length = snprintf(json, sizeof(json),
            "{\"record_type\":\"acoustic_diagnostic\",\"schema\":1,\"esp_ms\":%lu,"
            "\"feature_generation\":%lu,\"summary_valid\":%s,\"identifier_status\":%u,"
            "\"identifier_status_name\":\"%s\",\"reason\":\"%s\",\"distance\":%.3f,"
            "\"threshold\":%.3f,\"target_peak_bin\":%d,\"current_peak_bin\":%d,"
            "\"match_state\":\"%s\",\"strong_mismatch_count\":%u,\"last_target_age_ms\":%ld,"
            "\"cpu_diagnostic_drops\":%u,\"udp_diagnostic_drops\":%lu,\"summary\":[",
            (unsigned long) wifi_telemetry_uptime_ms(), (unsigned long) event->generation,
            (valid && ((snapshot->flags & ACOUSTIC_AI_LAB_FLAG_SUMMARY_VALID) != 0U)) ? "true" : "false",
            valid ? (unsigned int) snapshot->infer_status : 0U,
            valid ? wifi_telemetry_infer_status(snapshot->infer_status) : "UNKNOWN", reason,
            (double) distance, (double) threshold,
            (valid && snapshot->target_peak_bin != 255U) ? (int) snapshot->target_peak_bin : -1,
            (valid && snapshot->current_peak_bin != 255U) ? (int) snapshot->current_peak_bin : -1,
            match_name, (unsigned int) strong_count,
            (long) ((age_ms == UINT32_MAX) ? -1 : (int32_t) age_ms), (unsigned int) event->cpu_drop_count,
            (unsigned long) s_diagnostic_drop_count);
        if ((length > 0) && ((size_t) length < sizeof(json))) {
            for (size_t index = 0U; ((size_t) length < sizeof(json)) && (index < sizeof(event->vector)); index++) {
                int const appended = snprintf(&json[length], sizeof(json) - (size_t) length,
                                              "%s%d", (index == 0U) ? "" : ",", (int) event->vector[index]);
                if (appended <= 0 || ((size_t) appended >= sizeof(json) - (size_t) length)) {
                    length = (int) sizeof(json);
                } else {
                    length += appended;
                }
            }
            if ((size_t) length < sizeof(json)) {
                int const appended = snprintf(&json[length], sizeof(json) - (size_t) length,
                                              "]}\n");
                if (appended <= 0 || ((size_t) appended >= sizeof(json) - (size_t) length)) {
                    length = (int) sizeof(json);
                } else {
                    length += appended;
                }
            }
        }
    } else {
        length = snprintf(json, sizeof(json),
            "{\"record_type\":\"acoustic_sample\",\"schema\":1,\"esp_ms\":%lu,"
            "\"profile_generation\":%lu,\"sample_index\":%u,\"sample_count\":%u,"
            "\"udp_diagnostic_drops\":%lu,\"summary\":[",
            (unsigned long) wifi_telemetry_uptime_ms(), (unsigned long) event->generation,
            (unsigned int) event->sample_index, (unsigned int) event->sample_count,
            (unsigned long) s_diagnostic_drop_count);
        if ((length > 0) && ((size_t) length < sizeof(json))) {
            for (size_t index = 0U; ((size_t) length < sizeof(json)) && (index < sizeof(event->vector)); index++) {
                int const appended = snprintf(&json[length], sizeof(json) - (size_t) length,
                                              "%s%d", (index == 0U) ? "" : ",", (int) event->vector[index]);
                if (appended <= 0 || ((size_t) appended >= sizeof(json) - (size_t) length)) {
                    length = (int) sizeof(json);
                } else {
                    length += appended;
                }
            }
            if ((size_t) length < sizeof(json)) {
                int const appended = snprintf(&json[length], sizeof(json) - (size_t) length, "]}\n");
                if (appended <= 0 || ((size_t) appended >= sizeof(json) - (size_t) length)) {
                    length = (int) sizeof(json);
                } else {
                    length += appended;
                }
            }
        }
    }

    if ((length <= 0) || ((size_t) length >= sizeof(json))) {
        s_diagnostic_drop_count++;
        s_diagnostic_queue_head = (uint8_t) ((s_diagnostic_queue_head + 1U) % WIFI_DIAGNOSTIC_QUEUE_CAPACITY);
        s_diagnostic_queue_count--;
        return false;
    }
    int const sent = sendto(socket_fd, json, (size_t) length, 0,
                            (struct sockaddr *) &s_destination, sizeof(s_destination));
    if (sent != length) {
        s_udp_error_count++;
        return false;
    }
    s_udp_send_count++;
    s_diagnostic_queue_head = (uint8_t) ((s_diagnostic_queue_head + 1U) % WIFI_DIAGNOSTIC_QUEUE_CAPACITY);
    s_diagnostic_queue_count--;
    return true;
}

/** =================================================================*
 * @brief  Wi-Fi/IPイベント処理
 * @param[in] argument 登録時のユーザーコンテキスト
 * @param[in] event_base ESP-IDFイベントベース
 * @param[in] event_id ESP-IDFイベントID
 * @param[in] event_data ESP-IDFイベント情報
 * ================================================================= */
static void wifi_telemetry_event_handler(void * argument, esp_event_base_t event_base, int32_t event_id,
                                         void * event_data) {
    (void) argument;
    (void) event_data;

    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_START)) {
        (void) esp_wifi_connect();
    } else if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_DISCONNECTED)) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_TELEMETRY_CONNECTED_BIT);
        s_wifi_reconnect_count++;
        (void) esp_wifi_connect();
    } else if ((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP)) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_TELEMETRY_CONNECTED_BIT);
    }
}

/** =================================================================*
 * @brief  Wi-Fiステーション開始
 * @return NVS、ネットワーク、Wi-Fi初期化結果
 * ================================================================= */
static esp_err_t wifi_telemetry_station_start(void) {
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
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
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_telemetry_event_handler, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_telemetry_event_handler, NULL);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t wifi_config = {0};
    (void) snprintf((char *) wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", APP_WIFI_SSID);
    (void) snprintf((char *) wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", APP_WIFI_PASSWORD);
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

/** =================================================================*
 * @brief  CPU0テレメトリーJSON整形
 * @param[out] json JSON格納先
 * @param[in] capacity JSON格納先容量[byte]
 * @param[in] telemetry CPU0から受信した状態
 * @param[in] frame 受信フレーム情報
 * @param[in] actuator_telemetry CPU1から返された実出力状態
 * @param[in] actuator_frame_valid 実出力フレーム受信済みならtrue
 * @param[in] pose_telemetry 姿勢推定テレメトリ
 * @param[in] pose_frame_valid 姿勢推定フレーム受信済みならtrue
 * @param[in] received_at_ms ESP32での受信時刻[ms]
 * @param[in] actuator_received_at_ms 実出力フレーム受信時刻[ms]
 * @param[in] pose_received_at_ms 姿勢推定フレーム受信時刻[ms]
 * @param[in] nav_diagnostics 音源位置・到着・回避診断
 * @param[in] nav_frame_valid 音源ナビ診断フレーム受信済みならtrue
 * @param[in] nav_received_at_ms 音源ナビ診断フレーム受信時刻[ms]
 * @param[in] rssi_dbm Wi-Fi受信強度[dBm]
 * @param[in] esp_audio ESP32S3の音声DSP診断値
 * @param[in] feature_fps_x100 特徴量生成レート[fps x100]
 * @return 整形した文字数。容量不足時はcapacity以上
 * ================================================================= */
static int wifi_telemetry_format_json(char * json, size_t capacity, acoustic_rover_telemetry_t const * telemetry,
                                      acoustic_frame_t const * frame,
                                      acoustic_actuator_telemetry_t const * actuator_telemetry,
                                      bool actuator_frame_valid,
                                      acoustic_pose_telemetry_t const * pose_telemetry,
                                      bool pose_frame_valid, uint32_t received_at_ms,
                                      uint32_t actuator_received_at_ms, uint32_t pose_received_at_ms,
                                      acoustic_nav_diagnostics_t const * nav_diagnostics,
                                      bool nav_frame_valid, uint32_t nav_received_at_ms,
                                      int rssi_dbm, audio_capture_snapshot_t const * esp_audio,
                                      uint32_t feature_fps_x100) {
    uint32_t const now_ms = wifi_telemetry_uptime_ms();
    uint8_t const flags = telemetry->flags;
    bool const actuator_valid = actuator_frame_valid && (actuator_telemetry->status_valid != 0U);
    uint32_t const actuator_age_ms = actuator_valid
                                         ? actuator_telemetry->actuator_status_age_ms +
                                               (now_ms - actuator_received_at_ms)
                                         : UINT32_MAX;
    bool const pose_valid = pose_frame_valid && ((pose_telemetry->flags & ACOUSTIC_POSE_FLAG_CALIBRATED) != 0U);
    uint32_t const pose_age_ms = pose_frame_valid ? (now_ms - pose_received_at_ms) : UINT32_MAX;
    bool const nav_valid = nav_frame_valid && (nav_diagnostics->schema_version >= 1U);
    uint32_t const nav_age_ms = nav_valid ? (now_ms - nav_received_at_ms) : UINT32_MAX;
    uint16_t const raw_doa_deg = nav_valid ? nav_diagnostics->raw_doa_deg : ACOUSTIC_PROTOCOL_DOA_INVALID;
    uint16_t const filtered_doa_deg = nav_valid ? nav_diagnostics->filtered_doa_deg : telemetry->doa_deg;
    uint8_t const doa_confidence = nav_valid ? nav_diagnostics->doa_confidence : 0U;

    uint8_t const raw_infer_status = telemetry->infer_status;
    uint8_t const infer_status = acoustic_infer_status_unpack_status(raw_infer_status);
    uint8_t const classifier_kind = acoustic_infer_status_unpack_classifier(raw_infer_status);
    bool const tflm_available = acoustic_infer_status_unpack_tflm_available(raw_infer_status);

    float const cosine_dist = (0xFFFFU == telemetry->infer_cosine_dist_x1000)
                                  ? -1.0f
                                  : ((float) telemetry->infer_cosine_dist_x1000 / 1000.0f);
    float const threshold = (float) telemetry->infer_threshold_x1000 / 1000.0f;
    float const similarity = (float) telemetry->infer_similarity_permille / 10.0f;
    int const nearest_sample = (255U == telemetry->infer_nearest_sample) ? -1 : (int) telemetry->infer_nearest_sample;
    int const target_peak = (255U == telemetry->infer_target_peak_bin) ? -1 : (int) telemetry->infer_target_peak_bin;
    int const current_peak = (255U == telemetry->infer_current_peak_bin) ? -1 : (int) telemetry->infer_current_peak_bin;

    return snprintf(
        json, capacity,
        "{\"schema\":2,\"esp_ms\":%lu,\"cpu_valid\":true,"
        "\"cpu_age_ms\":%lu,\"wifi\":{\"connected\":1,"
        "\"rssi_dbm\":%d,\"reconnects\":%lu},"
        "\"udp\":{\"sent\":%lu,\"errors\":%lu},"
        "\"esp_audio\":{\"valid\":%d,\"pcm_frames\":%lu,"
        "\"feature_frames\":%lu,\"feature_fps_x100\":%lu,"
        "\"ring_frames\":%u,\"self_test_pass\":%s,"
        "\"i2s_overruns\":%lu},"
        "\"usb\":{\"mounted\":%d,\"rx_drops\":%lu,"
        "\"cpu_ms\":%lu,\"cpu_seq\":%lu,\"state\":%u,"
        "\"configured\":%d,\"hello\":%d},"
        "\"audio\":{\"observation\":%d,\"sequence\":%lu,"
        "\"age_ms\":%lu,\"doa_deg\":%u,\"raw_doa_deg\":%u,"
        "\"filtered_doa_deg\":%u,\"confidence\":%u,"
        "\"level_dbfs_x100\":%d,\"peak_dbfs_x100\":%d,"
        "\"vad\":%u,\"xvf_status\":%u,"
        "\"doa_fallback\":%d,\"flags\":%u,"
        "\"crc_errors\":%lu},"
        "\"think\":{\"state\":%u,\"name\":\"%s\","
        "\"link_ready\":%d,\"new_observation\":%d,"
        "\"faults\":%lu,\"steering_deg\":%d},"
        "\"command\":{\"left_rpm\":%d,\"right_rpm\":%d,"
        "\"enable\":%d,\"emergency_stop\":%d,\"stale\":%d,"
        "\"target_age_ms\":%lu,\"sequence\":%lu,\"last_error\":%ld},"
        "\"sensors\":{\"mode\":%u,\"mode_name\":\"%s\",\"rule\":%u,\"rule_name\":\"%s\"," 
        "\"valid_flags\":%u,\"tof_mm\":[%u,%u,%u],\"accel_mg\":[%d,%d,%d],"
        "\"gyro_dps_x10\":[%d,%d,%d],\"error_flags\":%u,\"last_error\":%ld,\"age_ms\":%lu},"
         "\"learning\":{\"active\":%d,\"storage_valid\":%d,\"storage_result\":%u,"
         "\"samples\":%u,\"threshold\":%.3f,\"target_peak_bin\":%d},"
         "\"debug_motor\":{\"active\":%d},"
         "\"recognition\":{\"status\":%u,\"status_name\":\"%s\",\"classifier\":\"%s\","
        "\"classifier_kind\":%u,\"tflm_available\":%s,\"distance\":%.3f,"
        "\"threshold\":%.3f,\"similarity\":%.1f,\"active_frames\":%u,\"nearest_sample\":%d,"
        "\"current_peak_bin\":%d},"
        "\"actuator\":{\"valid\":%u,\"age_ms\":%lu,"
        "\"status_sequence\":%lu,\"applied_command_sequence\":%lu,"
        "\"faults\":%u,\"left_duty_permille\":%d,"
        "\"right_duty_permille\":%d,\"left_encoder_rpm_x10\":%d,"
        "\"right_encoder_rpm_x10\":%d},"
        "\"pose\":{\"valid\":%u,\"age_ms\":%lu,"
        "\"x_mm\":%ld,\"y_mm\":%ld,\"theta_mrad\":%ld,"
        "\"v_mm_s\":%ld,\"omega_mrad_s\":%ld,"
        "\"left_encoder\":%ld,\"right_encoder\":%ld,"
        "\"gyro_bias_dps_x10\":%d,\"stationary\":%u,\"calibrated\":%u,"
        "\"uptime_ms\":%lu},"
        "\"navigation\":{\"valid\":%d,\"age_ms\":%lu,"
        "\"source_valid\":%d,"
        "\"observation_sequence\":%lu,\"rover_x_mm\":%ld,\"rover_y_mm\":%ld,"
        "\"rover_heading_mrad\":%ld,\"source_x_mm\":%ld,\"source_y_mm\":%ld,"
        "\"range_mm\":%lu,\"bearing_deg\":%d,\"confidence\":%u,"
        "\"observation_count\":%u,\"residual_mm\":%u,\"crossing_deg\":%u,"
        "\"baseline_mm\":%u,\"position_shift_mm\":%u,\"geometry_valid\":%d,"
        "\"target_valid\":%d},"
        "\"arrival\":{\"candidate\":%d,\"state\":%u,\"confirm_count\":%u},"
        "\"escape\":{\"minimum_translation_turn\":%d,\"sensor_rule\":%u,"
        "\"backup_command_count\":%lu}}\n",
        (unsigned long) now_ms, (unsigned long) (now_ms - received_at_ms), rssi_dbm,
        (unsigned long) s_wifi_reconnect_count, (unsigned long) s_udp_send_count, (unsigned long) s_udp_error_count,
        esp_audio->valid ? 1 : 0, (unsigned long) esp_audio->frame_count,
        (unsigned long) esp_audio->feature_frame_count, (unsigned long) feature_fps_x100,
        (unsigned int) esp_audio->feature_ring_frames, esp_audio->log_mel_self_test_pass ? "true" : "false",
        (unsigned long) esp_audio->overrun_count,
        usb_link_is_mounted() ? 1 : 0, (unsigned long) usb_link_rx_drop_count(), (unsigned long) frame->uptime_ms,
        (unsigned long) frame->sequence, (unsigned int) telemetry->usb_state,
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_USB_CONFIGURED),
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_HELLO_RECEIVED),
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_OBSERVATION),
        (unsigned long) telemetry->observation_sequence, (unsigned long) telemetry->observation_age_ms,
        (unsigned int) telemetry->doa_deg, (unsigned int) raw_doa_deg, (unsigned int) filtered_doa_deg,
        (unsigned int) doa_confidence, telemetry->level_dbfs_x100, telemetry->peak_dbfs_x100,
        (unsigned int) telemetry->vad, (unsigned int) telemetry->xvf_status,
        wifi_telemetry_flag(telemetry->audio_flags, ACOUSTIC_AUDIO_FLAG_DOA_FALLBACK),
        (unsigned int) telemetry->audio_flags,
        (unsigned long) telemetry->audio_crc_error_count, (unsigned int) telemetry->think_state,
        wifi_telemetry_think_state(telemetry->think_state),
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_LINK_READY),
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_NEW_OBSERVATION), (unsigned long) telemetry->fault_flags,
        telemetry->steering_deg, telemetry->left_target_rpm, telemetry->right_target_rpm,
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_ACTUATOR_ENABLE),
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_EMERGENCY_STOP),
        wifi_telemetry_flag(flags, ACOUSTIC_TELEMETRY_FLAG_COMMAND_STALE),
        (unsigned long) telemetry->command_target_age_ms, (unsigned long) telemetry->command_sequence,
        (long) telemetry->command_last_error,
        (unsigned int) telemetry->autonomy_mode, wifi_telemetry_autonomy_mode(telemetry->autonomy_mode),
        (unsigned int) telemetry->sensor_rule, wifi_telemetry_sensor_rule(telemetry->sensor_rule),
        (unsigned int) telemetry->sensor_valid_flags, (unsigned int) telemetry->tof_distance_mm[0],
        (unsigned int) telemetry->tof_distance_mm[1], (unsigned int) telemetry->tof_distance_mm[2],
        telemetry->accel_mg[0], telemetry->accel_mg[1], telemetry->accel_mg[2], telemetry->gyro_dps_x10[0],
        telemetry->gyro_dps_x10[1], telemetry->gyro_dps_x10[2], (unsigned int) telemetry->sensor_error_flags,
        (long) telemetry->sensor_last_error, (unsigned long) telemetry->sensor_age_ms,
        wifi_telemetry_flag(telemetry->sensor_reserved, ACOUSTIC_TELEMETRY_LEARNING_MODE),
        wifi_telemetry_flag(telemetry->sensor_reserved, ACOUSTIC_TELEMETRY_STORAGE_VALID),
         (unsigned int) (telemetry->sensor_reserved & ACOUSTIC_TELEMETRY_STORAGE_RESULT_MASK),
         (unsigned int) telemetry->infer_sample_count, (double) threshold, target_peak,
         wifi_telemetry_flag(telemetry->sensor_reserved, ACOUSTIC_TELEMETRY_DEBUG_MOTOR_RECORDING),
         (unsigned int) infer_status, wifi_telemetry_infer_status(infer_status),
         wifi_telemetry_classifier_name(classifier_kind), (unsigned int) classifier_kind,
         tflm_available ? "true" : "false",
        (double) cosine_dist, (double) threshold, (double) similarity,
        (unsigned int) telemetry->infer_active_frames, nearest_sample, current_peak,
        actuator_valid ? 1U : 0U, (unsigned long) actuator_age_ms,
        (unsigned long) actuator_telemetry->actuator_status_sequence,
        (unsigned long) actuator_telemetry->actuator_applied_command_sequence,
        (unsigned int) actuator_telemetry->fault_flags, actuator_telemetry->actuator_left_duty_permille,
        actuator_telemetry->actuator_right_duty_permille, actuator_telemetry->actuator_left_encoder_rpm_x10,
        actuator_telemetry->actuator_right_encoder_rpm_x10,
        pose_valid ? 1U : 0U, (unsigned long) pose_age_ms,
        (long) pose_telemetry->x_mm, (long) pose_telemetry->y_mm, (long) pose_telemetry->theta_mrad,
        (long) pose_telemetry->v_mm_s, (long) pose_telemetry->omega_mrad_s,
        (long) pose_telemetry->left_encoder_count, (long) pose_telemetry->right_encoder_count,
        (int) pose_telemetry->gyro_bias_dps_x10,
        ((pose_telemetry->flags & ACOUSTIC_POSE_FLAG_STATIONARY) != 0U) ? 1U : 0U,
        ((pose_telemetry->flags & ACOUSTIC_POSE_FLAG_CALIBRATED) != 0U) ? 1U : 0U,
        (unsigned long) pose_telemetry->uptime_ms,
        nav_valid ? 1 : 0, (unsigned long) nav_age_ms,
        nav_valid && ((nav_diagnostics->flags & ACOUSTIC_NAV_FLAG_SOURCE_VALID) != 0U) ? 1 : 0,
        (unsigned long) (nav_valid ? nav_diagnostics->observation_sequence : telemetry->observation_sequence),
        (long) (nav_valid ? nav_diagnostics->rover_x_mm : 0),
        (long) (nav_valid ? nav_diagnostics->rover_y_mm : 0),
        (long) (nav_valid ? nav_diagnostics->rover_heading_mrad : 0),
        (long) (nav_valid ? nav_diagnostics->source_x_mm : 0),
        (long) (nav_valid ? nav_diagnostics->source_y_mm : 0),
        (unsigned long) (nav_valid ? nav_diagnostics->source_range_mm : 0U),
        (int) (nav_valid ? nav_diagnostics->source_bearing_deg : 0),
        (unsigned int) (nav_valid ? nav_diagnostics->source_confidence : 0U),
        (unsigned int) (nav_valid ? nav_diagnostics->observation_count : 0U),
        (unsigned int) (nav_valid ? nav_diagnostics->localization_residual_mm : 0U),
        (unsigned int) (nav_valid ? nav_diagnostics->crossing_angle_deg : 0U),
        (unsigned int) (nav_valid ? nav_diagnostics->baseline_mm : 0U),
        (unsigned int) (nav_valid ? nav_diagnostics->source_position_shift_mm : 0U),
        nav_valid && ((nav_diagnostics->flags & ACOUSTIC_NAV_FLAG_GEOMETRY_VALID) != 0U) ? 1 : 0,
        nav_valid && ((nav_diagnostics->flags & ACOUSTIC_NAV_FLAG_TARGET_VALID) != 0U) ? 1 : 0,
        nav_valid && ((nav_diagnostics->flags & ACOUSTIC_NAV_FLAG_ARRIVAL_CANDIDATE) != 0U) ? 1 : 0,
        (unsigned int) (nav_valid ? nav_diagnostics->arrival_state : 0U),
        (unsigned int) (nav_valid ? nav_diagnostics->arrival_confirm_count : 0U),
        nav_valid && ((nav_diagnostics->flags & ACOUSTIC_NAV_FLAG_MIN_TRANSLATION_TURN) != 0U) ? 1 : 0,
        (unsigned int) (nav_valid ? nav_diagnostics->sensor_rule : 0U),
        (unsigned long) (nav_valid ? nav_diagnostics->autonomous_backup_count : 0U));
}

/** =================================================================*
 * @brief  接続状態JSON整形
 * @param[out] json JSON格納先
 * @param[in] capacity JSON格納先容量[byte]
 * @param[in] rssi_dbm Wi-Fi受信強度[dBm]
 * @param[in] esp_audio ESP32S3の音声DSP診断値
 * @param[in] feature_fps_x100 特徴量生成レート[fps x100]
 * @return 整形した文字数。容量不足時はcapacity以上
 * ================================================================= */
static int wifi_telemetry_format_heartbeat(char * json, size_t capacity, int rssi_dbm,
                                           audio_capture_snapshot_t const * esp_audio,
                                           uint32_t feature_fps_x100) {
    return snprintf(json, capacity,
                    "{\"schema\":2,\"esp_ms\":%lu,\"cpu_valid\":false,"
                    "\"wifi\":{\"connected\":1,\"rssi_dbm\":%d,"
                    "\"reconnects\":%lu},\"udp\":{\"sent\":%lu,"
                    "\"errors\":%lu},\"usb\":{\"mounted\":%d,"
                    "\"rx_drops\":%lu},"
                    "\"esp_audio\":{\"valid\":%d,\"pcm_frames\":%lu,"
                    "\"feature_frames\":%lu,\"feature_fps_x100\":%lu,"
                    "\"ring_frames\":%u,\"self_test_pass\":%s,"
                    "\"i2s_overruns\":%lu}}\n",
                    (unsigned long) wifi_telemetry_uptime_ms(), rssi_dbm, (unsigned long) s_wifi_reconnect_count,
                    (unsigned long) s_udp_send_count, (unsigned long) s_udp_error_count, usb_link_is_mounted() ? 1 : 0,
                    (unsigned long) usb_link_rx_drop_count(), esp_audio->valid ? 1 : 0,
                    (unsigned long) esp_audio->frame_count, (unsigned long) esp_audio->feature_frame_count,
                    (unsigned long) feature_fps_x100, (unsigned int) esp_audio->feature_ring_frames,
                    esp_audio->log_mel_self_test_pass ? "true" : "false",
                    (unsigned long) esp_audio->overrun_count);
}

/** =================================================================*
 * @brief  USB受信・UDP送信タスク
 * @param[in] argument FreeRTOSタスク引数
 * @details CPU0からの状態を最新値として保持し、Wi-Fi接続中にUDPで送信する。
 * ================================================================= */
static void wifi_telemetry_task(void * argument) {
    (void) argument;

    acoustic_protocol_parser_t parser;
    acoustic_rover_telemetry_t latest_telemetry = {0};
    acoustic_actuator_telemetry_t latest_actuator_telemetry = {0};
    acoustic_pose_telemetry_t latest_pose_telemetry = {0};
    acoustic_nav_diagnostics_t latest_nav_diagnostics = {0};
    acoustic_ai_lab_snapshot_t latest_ai_lab_snapshot = {0};
    acoustic_frame_t latest_frame = {0};
    uint32_t received_at_ms = 0U;
    uint32_t actuator_received_at_ms = 0U;
    uint32_t pose_received_at_ms = 0U;
    uint32_t nav_received_at_ms = 0U;
    uint32_t last_send_ms = 0U;
    uint32_t previous_feature_frame_count = 0U;
    uint32_t previous_feature_sample_ms = 0U;
    bool telemetry_valid = false;
    bool actuator_frame_valid = false;
    bool pose_frame_valid = false;
    bool nav_frame_valid = false;
    bool ai_lab_snapshot_valid = false;
    int socket_fd = -1;
    uint8_t rx_data[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    acoustic_protocol_parser_init(&parser);

    while (true) {
        size_t received_length = 0U;
        esp_err_t const rx_err =
            usb_link_receive(rx_data, sizeof(rx_data), &received_length, APP_TELEMETRY_USB_POLL_MS);
        if (rx_err == ESP_OK) {
            for (size_t index = 0U; index < received_length; index++) {
                acoustic_frame_t frame;
                acoustic_parse_result_t const parse_result =
                    acoustic_protocol_parser_push(&parser, rx_data[index], &frame);
                if (parse_result == ACOUSTIC_PARSE_FRAME_READY) {
                    if (acoustic_protocol_decode_rover_telemetry(&frame, &latest_telemetry)) {
                        latest_frame = frame;
                        received_at_ms = wifi_telemetry_uptime_ms();
                        telemetry_valid = true;
                    } else if (acoustic_protocol_decode_actuator_telemetry(&frame, &latest_actuator_telemetry)) {
                        actuator_received_at_ms = wifi_telemetry_uptime_ms();
                        actuator_frame_valid = true;
                    } else if (acoustic_protocol_decode_pose_telemetry(&frame, &latest_pose_telemetry)) {
                        pose_received_at_ms = wifi_telemetry_uptime_ms();
                        pose_frame_valid = true;
                    } else if (acoustic_protocol_decode_nav_diagnostics(&frame, &latest_nav_diagnostics)) {
                        nav_received_at_ms = wifi_telemetry_uptime_ms();
                        nav_frame_valid = true;
                    } else if (acoustic_protocol_decode_ai_lab_snapshot(&frame, &latest_ai_lab_snapshot)) {
                        ai_lab_snapshot_valid = true;
                    } else {
                        acoustic_ai_lab_summary_chunk_t summary_chunk = {0};
                        acoustic_ai_lab_profile_chunk_t profile_chunk = {0};
                        if (acoustic_protocol_decode_ai_lab_summary_chunk(&frame, &summary_chunk)) {
                            (void) wifi_telemetry_summary_chunk_accept(&summary_chunk, &latest_ai_lab_snapshot,
                                                                       ai_lab_snapshot_valid);
                        } else if (acoustic_protocol_decode_ai_lab_profile_chunk(&frame, &profile_chunk)) {
                            (void) wifi_telemetry_profile_chunk_accept(&profile_chunk);
                        }
                    }
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
        if (socket_fd < 0) {
            socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (socket_fd < 0) {
                s_udp_error_count++;
                continue;
            }
        }

#if APP_AUDIO_DATASET_STREAM_ENABLE
        wifi_telemetry_pcm_send(socket_fd);
#endif
        if ((now_ms - last_send_ms) < APP_UDP_TELEMETRY_PERIOD_MS) {
            continue;
        }
        last_send_ms = now_ms;

        wifi_ap_record_t access_point = {0};
        int rssi_dbm = -127;
        if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK) {
            rssi_dbm = access_point.rssi;
        }

        audio_capture_snapshot_t esp_audio = {0};
        audio_capture_get_snapshot(&esp_audio);
        uint32_t feature_fps_x100 = 0U;
        if ((previous_feature_sample_ms != 0U) && (now_ms != previous_feature_sample_ms)) {
            uint32_t const elapsed_ms = now_ms - previous_feature_sample_ms;
            uint32_t const generated_frames = esp_audio.feature_frame_count - previous_feature_frame_count;
            feature_fps_x100 = (uint32_t) (((uint64_t) generated_frames * 100000ULL) / elapsed_ms);
        }
        previous_feature_frame_count = esp_audio.feature_frame_count;
        previous_feature_sample_ms = now_ms;

        static char s_telemetry_json[WIFI_TELEMETRY_JSON_CAPACITY];
        int const json_length = telemetry_valid
                                    ? wifi_telemetry_format_json(
                                          s_telemetry_json, sizeof(s_telemetry_json), &latest_telemetry, &latest_frame,
                                          &latest_actuator_telemetry, actuator_frame_valid,
                                          &latest_pose_telemetry, pose_frame_valid,
                                          received_at_ms, actuator_received_at_ms, pose_received_at_ms,
                                          &latest_nav_diagnostics, nav_frame_valid, nav_received_at_ms,
                                          rssi_dbm, &esp_audio, feature_fps_x100)
                                    : wifi_telemetry_format_heartbeat(s_telemetry_json, sizeof(s_telemetry_json), rssi_dbm, &esp_audio,
                                                                      feature_fps_x100);
        if ((json_length <= 0) || ((size_t) json_length >= sizeof(s_telemetry_json))) {
            s_udp_error_count++;
            continue;
        }

        int const sent =
            sendto(socket_fd, s_telemetry_json, (size_t) json_length, 0, (struct sockaddr *) &s_destination, sizeof(s_destination));
        if (sent == json_length) {
            s_udp_send_count++;
        } else {
            s_udp_error_count++;
        }
        (void) wifi_telemetry_diagnostic_send(socket_fd);
    }
}

/** =================================================================*
 * @brief  Wi-Fi接続状態取得
 * @return IPv4アドレスを取得済みならtrue
 * ================================================================= */
bool wifi_telemetry_is_connected(void) {
    if (s_wifi_event_group == NULL) {
        return false;
    }
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_TELEMETRY_CONNECTED_BIT) != 0U;
}

/** =================================================================*
 * @brief  Wi-Fiテレメトリー送信開始
 * @return 設定検査、Wi-Fi初期化またはタスク生成結果
 * ================================================================= */
esp_err_t wifi_telemetry_start(void) {
    if ((APP_WIFI_SSID[0] == '\0') || (APP_UDP_DESTINATION_IPV4[0] == '\0') ||
        (strlen(APP_WIFI_SSID) >= sizeof(((wifi_config_t *) 0)->sta.ssid)) ||
        (strlen(APP_WIFI_PASSWORD) >= sizeof(((wifi_config_t *) 0)->sta.password))) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_destination, 0, sizeof(s_destination));
    s_destination.sin_family = AF_INET;
    s_destination.sin_port = htons(APP_UDP_DESTINATION_PORT);
    if (inet_pton(AF_INET, APP_UDP_DESTINATION_IPV4, &s_destination.sin_addr) != 1) {
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

    BaseType_t const task_result = xTaskCreate(wifi_telemetry_task, "wifi_telemetry", APP_TELEMETRY_TASK_STACK_SIZE,
                                               NULL, APP_TELEMETRY_TASK_PRIORITY, NULL);
    return (task_result == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
