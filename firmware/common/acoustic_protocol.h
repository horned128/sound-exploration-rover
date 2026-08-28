/** =================================================================*
 * @file   acoustic_protocol.h
 * @brief  ReSpeaker・RA8P1間音響通信プロトコル
 * ================================================================= */
#ifndef SEROV_ACOUSTIC_PROTOCOL_H
#define SEROV_ACOUSTIC_PROTOCOL_H

#include <stdbool.h>                                       /* 真偽値 */
#include <stddef.h>                                        /* size_t */
#include <stdint.h>                                        /* 固定幅整数型 */

#define ACOUSTIC_PROTOCOL_MAGIC_0                 (0x53U)
#define ACOUSTIC_PROTOCOL_MAGIC_1                 (0x52U)
#define ACOUSTIC_PROTOCOL_VERSION                 (1U)
#define ACOUSTIC_PROTOCOL_HEADER_SIZE             (14U)
#define ACOUSTIC_PROTOCOL_CRC_SIZE                (2U)
#define ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE        (64U)
#define ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE          \
    (ACOUSTIC_PROTOCOL_HEADER_SIZE +              \
     ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE +         \
     ACOUSTIC_PROTOCOL_CRC_SIZE)
#define ACOUSTIC_PROTOCOL_DOA_INVALID             (0xFFFFU)
#define ACOUSTIC_OBSERVATION_PAYLOAD_SIZE         (14U)
#define ACOUSTIC_HELLO_PAYLOAD_SIZE               (12U)
#define ACOUSTIC_HEALTH_PAYLOAD_SIZE              (12U)
#define ACOUSTIC_ROVER_TELEMETRY_PAYLOAD_SIZE     (64U)

#define ACOUSTIC_CAPABILITY_DOA                   (1UL << 0)
#define ACOUSTIC_CAPABILITY_VAD                   (1UL << 1)
#define ACOUSTIC_CAPABILITY_LEVEL                 (1UL << 2)
#define ACOUSTIC_CAPABILITY_WIFI                  (1UL << 3)

#define ACOUSTIC_AUDIO_FLAG_I2S_OVERRUN           (1U << 0)
#define ACOUSTIC_AUDIO_FLAG_I2C_ERROR             (1U << 1)
#define ACOUSTIC_AUDIO_FLAG_MUTED                 (1U << 2)
#define ACOUSTIC_AUDIO_FLAG_I2S_STALE             (1U << 3)
#define ACOUSTIC_AUDIO_FLAG_DOA_FALLBACK          (1U << 4)

#define ACOUSTIC_TELEMETRY_FLAG_USB_CONFIGURED    (1U << 0)
#define ACOUSTIC_TELEMETRY_FLAG_HELLO_RECEIVED    (1U << 1)
#define ACOUSTIC_TELEMETRY_FLAG_OBSERVATION       (1U << 2)
#define ACOUSTIC_TELEMETRY_FLAG_LINK_READY        (1U << 3)
#define ACOUSTIC_TELEMETRY_FLAG_NEW_OBSERVATION   (1U << 4)
#define ACOUSTIC_TELEMETRY_FLAG_ACTUATOR_ENABLE   (1U << 5)
#define ACOUSTIC_TELEMETRY_FLAG_EMERGENCY_STOP    (1U << 6)
#define ACOUSTIC_TELEMETRY_FLAG_COMMAND_STALE     (1U << 7)

typedef enum e_acoustic_message_type {
    ACOUSTIC_MESSAGE_HELLO       = 0x01U,
    ACOUSTIC_MESSAGE_OBSERVATION = 0x02U,
    ACOUSTIC_MESSAGE_HEALTH      = 0x03U,
    ACOUSTIC_MESSAGE_SET_CONFIG  = 0x10U,
    ACOUSTIC_MESSAGE_ACK         = 0x11U,
    ACOUSTIC_MESSAGE_ROVER_TELEMETRY = 0x20U,
    ACOUSTIC_MESSAGE_LOG         = 0x7FU,
} acoustic_message_type_t;

typedef enum e_acoustic_xvf_status {
    ACOUSTIC_XVF_STATUS_STARTING = 0U,
    ACOUSTIC_XVF_STATUS_READY    = 1U,
    ACOUSTIC_XVF_STATUS_ERROR    = 2U,
} acoustic_xvf_status_t;

typedef struct st_acoustic_observation {
    uint16_t doa_deg;
    int16_t level_dbfs_x100;
    int16_t peak_dbfs_x100;
    uint8_t vad;
    uint8_t xvf_status;
    uint8_t audio_flags;
    uint8_t xvf_raw_status;
    uint32_t audio_frame_count;
} acoustic_observation_t;

typedef struct st_acoustic_hello {
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t firmware_patch;
    uint8_t reserved;
    uint32_t capabilities;
    uint32_t boot_id;
} acoustic_hello_t;

typedef struct st_acoustic_health {
    uint8_t xvf_status;
    uint8_t audio_flags;
    uint8_t usb_connected;
    uint8_t wifi_connected;
    uint32_t i2c_error_count;
    uint32_t i2s_overrun_count;
} acoustic_health_t;

typedef struct st_acoustic_rover_telemetry {
    uint8_t schema_version;
    uint8_t think_state;
    uint8_t usb_state;
    uint8_t flags;
    uint16_t doa_deg;
    int16_t level_dbfs_x100;
    int16_t peak_dbfs_x100;
    uint8_t vad;
    uint8_t xvf_status;
    uint8_t audio_flags;
    uint8_t xvf_raw_status;
    uint32_t observation_sequence;
    uint32_t observation_age_ms;
    uint32_t audio_frame_count;
    uint32_t audio_crc_error_count;
    uint32_t fault_flags;
    int16_t steering_deg;
    int16_t left_target_rpm;
    int16_t right_target_rpm;
    int16_t servo_target_deg[4];
    uint32_t command_sequence;
    int32_t command_last_error;
    uint32_t command_target_age_ms;
    uint32_t command_send_count;
} acoustic_rover_telemetry_t;

typedef struct st_acoustic_frame {
    uint8_t version;
    acoustic_message_type_t type;
    uint16_t payload_length;
    uint32_t sequence;
    uint32_t uptime_ms;
    uint8_t payload[ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE];
} acoustic_frame_t;

typedef struct st_acoustic_protocol_parser {
    uint8_t bytes[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t used;
    uint16_t expected;
} acoustic_protocol_parser_t;

typedef enum e_acoustic_parse_result {
    ACOUSTIC_PARSE_MORE = 0,
    ACOUSTIC_PARSE_FRAME_READY,
    ACOUSTIC_PARSE_CRC_ERROR,
    ACOUSTIC_PARSE_FORMAT_ERROR,
    ACOUSTIC_PARSE_UNSUPPORTED_VERSION,
} acoustic_parse_result_t;

uint16_t acoustic_protocol_crc16(const uint8_t * p_data,
                                 size_t length);            /* CRC-16/CCITT-FALSE */
size_t acoustic_protocol_encode(acoustic_message_type_t type,
                                uint32_t sequence,
                                uint32_t uptime_ms,
                                const uint8_t * p_payload,
                                uint16_t payload_length,
                                uint8_t * p_output,
                                size_t output_capacity);    /* 汎用フレーム符号化 */
size_t acoustic_protocol_encode_observation(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_observation_t * p_observation,
    uint8_t * p_output,
    size_t output_capacity);                               /* 音響観測符号化 */
size_t acoustic_protocol_encode_hello(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_hello_t * p_hello,
    uint8_t * p_output,
    size_t output_capacity);                               /* 起動情報符号化 */
size_t acoustic_protocol_encode_health(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_health_t * p_health,
    uint8_t * p_output,
    size_t output_capacity);                               /* 健全性情報符号化 */
size_t acoustic_protocol_encode_rover_telemetry(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_rover_telemetry_t * p_telemetry,
    uint8_t * p_output,
    size_t output_capacity);                               /* ローバ診断符号化 */
void acoustic_protocol_parser_init(
    acoustic_protocol_parser_t * p_parser);                /* パーサー初期化 */
acoustic_parse_result_t acoustic_protocol_parser_push(
    acoustic_protocol_parser_t * p_parser,
    uint8_t byte,
    acoustic_frame_t * p_frame);                           /* 1 byte受信 */
bool acoustic_protocol_decode_observation(
    const acoustic_frame_t * p_frame,
    acoustic_observation_t * p_observation);               /* 音響観測復号 */
bool acoustic_protocol_decode_hello(
    const acoustic_frame_t * p_frame,
    acoustic_hello_t * p_hello);                           /* 起動情報復号 */
bool acoustic_protocol_decode_health(
    const acoustic_frame_t * p_frame,
    acoustic_health_t * p_health);                         /* 健全性情報復号 */
bool acoustic_protocol_decode_rover_telemetry(
    const acoustic_frame_t * p_frame,
    acoustic_rover_telemetry_t * p_telemetry);             /* ローバ診断復号 */

#endif /* SEROV_ACOUSTIC_PROTOCOL_H */
