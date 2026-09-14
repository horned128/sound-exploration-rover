/** =================================================================*
 * @file   acoustic_protocol.h
 * @brief  ReSpeaker・RA8P1間音響通信プロトコル
 * ================================================================= */
#ifndef SEROV_ACOUSTIC_PROTOCOL_H
#define SEROV_ACOUSTIC_PROTOCOL_H

#include <stdbool.h>                                        /* 真偽値 */
#include <stddef.h>                                         /* size_t */
#include <stdint.h>                                         /* 固定幅整数型 */

#define ACOUSTIC_PROTOCOL_MAGIC_0          (0x53U)
#define ACOUSTIC_PROTOCOL_MAGIC_1          (0x52U)
#define ACOUSTIC_PROTOCOL_VERSION          (1U)
#define ACOUSTIC_PROTOCOL_HEADER_SIZE      (14U)
#define ACOUSTIC_PROTOCOL_CRC_SIZE         (2U)
#define ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE (96U)
#define ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE   \
    (ACOUSTIC_PROTOCOL_HEADER_SIZE + ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE + ACOUSTIC_PROTOCOL_CRC_SIZE)
#define ACOUSTIC_PROTOCOL_DOA_INVALID         (0xFFFFU)
#define ACOUSTIC_OBSERVATION_PAYLOAD_SIZE     (14U)
#define ACOUSTIC_HELLO_PAYLOAD_SIZE           (12U)
#define ACOUSTIC_HEALTH_PAYLOAD_SIZE          (12U)
#define ACOUSTIC_FEATURE_BIN_COUNT             (32U)
#define ACOUSTIC_FEATURE_FRAMES_PER_PACKET     (2U)
#define ACOUSTIC_FEATURE_EVENT_FRAME_COUNT     (80U)
#define ACOUSTIC_FEATURE_META_SIZE             (8U)
#define ACOUSTIC_FEATURE_DATA_SIZE             \
    (ACOUSTIC_FEATURE_BIN_COUNT * ACOUSTIC_FEATURE_FRAMES_PER_PACKET)
#define ACOUSTIC_FEATURE_PAYLOAD_SIZE          (ACOUSTIC_FEATURE_META_SIZE + ACOUSTIC_FEATURE_DATA_SIZE)
#define ACOUSTIC_ROVER_TELEMETRY_PAYLOAD_SIZE (96U)
#define ACOUSTIC_ACTUATOR_TELEMETRY_PAYLOAD_SIZE (24U)

#define ACOUSTIC_CAPABILITY_DOA            (1UL << 0)
#define ACOUSTIC_CAPABILITY_VAD            (1UL << 1)
#define ACOUSTIC_CAPABILITY_LEVEL          (1UL << 2)
#define ACOUSTIC_CAPABILITY_WIFI           (1UL << 3)

#define ACOUSTIC_AUDIO_FLAG_I2S_OVERRUN    (1U << 0)
#define ACOUSTIC_AUDIO_FLAG_I2C_ERROR      (1U << 1)
#define ACOUSTIC_AUDIO_FLAG_MUTED          (1U << 2)
#define ACOUSTIC_AUDIO_FLAG_I2S_STALE      (1U << 3)
#define ACOUSTIC_AUDIO_FLAG_DOA_FALLBACK   (1U << 4)

#define ACOUSTIC_TELEMETRY_FLAG_USB_CONFIGURED  (1U << 0)
#define ACOUSTIC_TELEMETRY_FLAG_HELLO_RECEIVED  (1U << 1)
#define ACOUSTIC_TELEMETRY_FLAG_OBSERVATION     (1U << 2)
#define ACOUSTIC_TELEMETRY_FLAG_LINK_READY      (1U << 3)
#define ACOUSTIC_TELEMETRY_FLAG_NEW_OBSERVATION (1U << 4)
#define ACOUSTIC_TELEMETRY_FLAG_ACTUATOR_ENABLE (1U << 5)
#define ACOUSTIC_TELEMETRY_FLAG_EMERGENCY_STOP  (1U << 6)
#define ACOUSTIC_TELEMETRY_FLAG_COMMAND_STALE   (1U << 7)

#define ACOUSTIC_TELEMETRY_STORAGE_RESULT_MASK (0x0FU)
#define ACOUSTIC_TELEMETRY_STORAGE_VALID   (1U << 4)
#define ACOUSTIC_TELEMETRY_LEARNING_MODE   (1U << 5)

typedef enum e_acoustic_message_type {
    ACOUSTIC_MESSAGE_HELLO = 0x01U,
    ACOUSTIC_MESSAGE_OBSERVATION = 0x02U,
    ACOUSTIC_MESSAGE_HEALTH = 0x03U,
    ACOUSTIC_MESSAGE_FEATURE = 0x04U,
    ACOUSTIC_MESSAGE_PROTOTYPE_DATA = 0x05U,
    ACOUSTIC_MESSAGE_SET_CONFIG = 0x10U,
    ACOUSTIC_MESSAGE_ACK = 0x11U,
    ACOUSTIC_MESSAGE_ROVER_TELEMETRY = 0x20U,
    ACOUSTIC_MESSAGE_ACTUATOR_TELEMETRY = 0x21U,
    ACOUSTIC_MESSAGE_POSE_TELEMETRY = 0x22U,
    ACOUSTIC_MESSAGE_LOG = 0x7FU,
} acoustic_message_type_t;

typedef enum e_acoustic_xvf_status {
    ACOUSTIC_XVF_STATUS_STARTING = 0U,
    ACOUSTIC_XVF_STATUS_READY = 1U,
    ACOUSTIC_XVF_STATUS_ERROR = 2U,
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

typedef struct st_acoustic_feature {
    uint16_t event_id;
    uint16_t frame_index;
    uint16_t frame_count;
    uint8_t n_bins;
    uint8_t flags;
    int8_t mel[ACOUSTIC_FEATURE_DATA_SIZE];
} acoustic_feature_t;

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
    uint8_t autonomy_mode;
    uint8_t sensor_rule;
    uint8_t sensor_valid_flags;
    uint8_t sensor_reserved;
    uint16_t tof_distance_mm[3];
    int16_t accel_mg[3];
    int16_t gyro_dps_x10[3];
    uint16_t sensor_error_flags;
    int32_t sensor_last_error;
    uint32_t sensor_age_ms;
} acoustic_rover_telemetry_t;

typedef struct st_acoustic_actuator_telemetry {
    uint8_t schema_version;
    uint8_t status_valid;
    uint16_t fault_flags;
    uint32_t actuator_status_age_ms;
    uint32_t actuator_status_sequence;
    uint32_t actuator_applied_command_sequence;
    int16_t actuator_left_duty_permille;
    int16_t actuator_right_duty_permille;
    int16_t actuator_left_encoder_rpm_x10;
    int16_t actuator_right_encoder_rpm_x10;
} acoustic_actuator_telemetry_t;

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

uint16_t acoustic_protocol_crc16(const uint8_t * p_data, size_t length); /* CRC-16/CCITT-FALSE */
size_t acoustic_protocol_encode(acoustic_message_type_t type, uint32_t sequence, uint32_t uptime_ms,
                                const uint8_t * p_payload, uint16_t payload_length, uint8_t * p_output,
                                size_t output_capacity);    /* 汎用フレーム符号化 */
size_t acoustic_protocol_encode_observation(uint32_t sequence, uint32_t uptime_ms,
                                            const acoustic_observation_t * p_observation, uint8_t * p_output,
                                                            /* 音響観測符号化 */
                                            size_t output_capacity);
size_t acoustic_protocol_encode_hello(uint32_t sequence, uint32_t uptime_ms, const acoustic_hello_t * p_hello,
                                                            /* 起動情報符号化 */
                                      uint8_t * p_output, size_t output_capacity);
size_t acoustic_protocol_encode_health(uint32_t sequence, uint32_t uptime_ms, const acoustic_health_t * p_health,
                                                            /* 健全性情報符号化 */
                                       uint8_t * p_output, size_t output_capacity);
size_t acoustic_protocol_encode_feature(uint32_t sequence, uint32_t uptime_ms,
                                        const acoustic_feature_t * p_feature, uint8_t * p_output,
                                        size_t output_capacity); /* log-mel特徴量符号化 */
size_t acoustic_protocol_encode_rover_telemetry(uint32_t sequence, uint32_t uptime_ms,
                                                const acoustic_rover_telemetry_t * p_telemetry, uint8_t * p_output,
                                                            /* ローバ診断符号化 */
                                                size_t output_capacity);
size_t acoustic_protocol_encode_actuator_telemetry(uint32_t sequence, uint32_t uptime_ms,
                                                   const acoustic_actuator_telemetry_t * p_telemetry,
                                                   uint8_t * p_output, size_t output_capacity);
void acoustic_protocol_parser_init(acoustic_protocol_parser_t * p_parser); /* パーサー初期化 */
acoustic_parse_result_t acoustic_protocol_parser_push(acoustic_protocol_parser_t * p_parser, uint8_t byte,
                                                            /* 1 byte受信 */
                                                      acoustic_frame_t * p_frame);
bool acoustic_protocol_decode_observation(const acoustic_frame_t * p_frame,
                                                            /* 音響観測復号 */
                                          acoustic_observation_t * p_observation);
bool acoustic_protocol_decode_hello(const acoustic_frame_t * p_frame, acoustic_hello_t * p_hello); /* 起動情報復号 */
bool acoustic_protocol_decode_health(const acoustic_frame_t * p_frame,
                                                            /* 健全性情報復号 */
                                     acoustic_health_t * p_health);
bool acoustic_protocol_decode_feature(const acoustic_frame_t * p_frame,
                                      acoustic_feature_t * p_feature); /* log-mel特徴量復号 */
bool acoustic_protocol_decode_rover_telemetry(const acoustic_frame_t * p_frame,
                                                            /* ローバ診断復号 */
                                              acoustic_rover_telemetry_t * p_telemetry);
bool acoustic_protocol_decode_actuator_telemetry(const acoustic_frame_t * p_frame,
                                                 acoustic_actuator_telemetry_t * p_telemetry);

#endif /* SEROV_ACOUSTIC_PROTOCOL_H */
