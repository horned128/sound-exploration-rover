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
#define ACOUSTIC_PROTOCOL_VERSION          (2U)
#define ACOUSTIC_PROTOCOL_HEADER_SIZE      (14U)
#define ACOUSTIC_PROTOCOL_CRC_SIZE         (2U)
#define ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE (96U)
#define ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE   \
    (ACOUSTIC_PROTOCOL_HEADER_SIZE + ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE + ACOUSTIC_PROTOCOL_CRC_SIZE)
#define ACOUSTIC_PROTOCOL_DOA_INVALID         (0xFFFFU)
#define ACOUSTIC_OBSERVATION_PAYLOAD_SIZE     (22U)
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
#define ACOUSTIC_POSE_TELEMETRY_PAYLOAD_SIZE (36U)
#define ACOUSTIC_NAV_DIAGNOSTICS_PAYLOAD_SIZE (56U)
#define ACOUSTIC_AI_LAB_SNAPSHOT_PAYLOAD_SIZE        (48U) /**< snapshot長[byte] */
#define ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE              (64U) /**< chunkデータ長[byte] */
#define ACOUSTIC_AI_LAB_SUMMARY_CHUNK_PAYLOAD_SIZE   (8U + ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE) /**< 要約chunk長 */
#define ACOUSTIC_AI_LAB_PROFILE_CHUNK_PAYLOAD_SIZE   (8U + ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE) /**< 保存見本chunk長 */
#define ACOUSTIC_AI_LAB_COMMAND_PAYLOAD_SIZE         (1U)  /**< 操作要求長[byte] */
#define ACOUSTIC_AI_LAB_COMMAND_RESULT_PAYLOAD_SIZE  (4U)  /**< 操作結果長[byte] */
#define ACOUSTIC_AI_LAB_MATCH_STATE_MASK             (0x03U) /**< snapshot v2 一致状態bit */
#define ACOUSTIC_AI_LAB_MATCH_CONFIRMED              (0U)    /**< TARGET確定 */
#define ACOUSTIC_AI_LAB_MATCH_UNCERTAIN              (1U)    /**< 期限付き猶予 */
#define ACOUSTIC_AI_LAB_MATCH_EXPIRED                (2U)    /**< 一致期限切れ */
#define ACOUSTIC_AI_LAB_MATCH_UNKNOWN                (3U)    /**< baseline制御では保持状態を提供しない */
#define ACOUSTIC_AI_LAB_MATCH_STRONG_COUNT_SHIFT     (2U)    /**< snapshot v2 強不一致数shift */
#define ACOUSTIC_AI_LAB_MATCH_STRONG_COUNT_MASK      (0xFCU) /**< snapshot v2 強不一致数mask */
#define ACOUSTIC_AI_LAB_MATCH_AGE_INVALID            (0xFFFFU) /**< TARGET未確認の経過時間 */

#define ACOUSTIC_POSE_FLAG_STATIONARY      (1U << 0)
#define ACOUSTIC_POSE_FLAG_CALIBRATED      (1U << 1)

#define ACOUSTIC_CAPABILITY_DOA            (1UL << 0)
#define ACOUSTIC_CAPABILITY_VAD            (1UL << 1)
#define ACOUSTIC_CAPABILITY_LEVEL          (1UL << 2)
#define ACOUSTIC_CAPABILITY_WIFI           (1UL << 3)
#define ACOUSTIC_CAPABILITY_DOA_DIAGNOSTICS (1UL << 4)
#define ACOUSTIC_CAPABILITY_FEATURE_SLOT1   (1UL << 5) /**< DSPはI2S右slot。CPU0 v5見本に必須 */

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
#define ACOUSTIC_TELEMETRY_DEBUG_MOTOR_RECORDING (1U << 6) /**< SW1録音走行（診断のみ） */

#define ACOUSTIC_INFER_STATUS_MASK             (0x0FU)     /**< 認識判定状態 (0:INVALID..4:TARGET) */
#define ACOUSTIC_INFER_CLASSIFIER_SHIFT        (4U)        /**< 分類器種別ビットシフト */
#define ACOUSTIC_INFER_CLASSIFIER_MASK         (0x30U)     /**< 分類器種別マスク (2bit) */
#define ACOUSTIC_INFER_CLASSIFIER_NONE         (0U)        /**< 未判定 */
#define ACOUSTIC_INFER_CLASSIFIER_DSP_SUMMARY  (1U)        /**< 192次元要約・周波数ビン直接照合 */
#define ACOUSTIC_INFER_CLASSIFIER_NN_EMBEDDING (2U)        /**< TFLM音響埋め込みCNN照合 */
#define ACOUSTIC_INFER_TFLM_AVAILABLE_BIT      (1U << 6)   /**< TFLMランタイム初期化成功フラグ */

static inline uint8_t acoustic_infer_status_pack(uint8_t status, uint8_t classifier_kind, bool tflm_available) {
    return (uint8_t) ((status & ACOUSTIC_INFER_STATUS_MASK) |
                      ((classifier_kind & 0x03U) << ACOUSTIC_INFER_CLASSIFIER_SHIFT) |
                      (tflm_available ? ACOUSTIC_INFER_TFLM_AVAILABLE_BIT : 0U));
}

static inline uint8_t acoustic_infer_status_unpack_status(uint8_t packed) {
    return (uint8_t) (packed & ACOUSTIC_INFER_STATUS_MASK);
}

static inline uint8_t acoustic_infer_status_unpack_classifier(uint8_t packed) {
    return (uint8_t) ((packed & ACOUSTIC_INFER_CLASSIFIER_MASK) >> ACOUSTIC_INFER_CLASSIFIER_SHIFT);
}

static inline bool acoustic_infer_status_unpack_tflm_available(uint8_t packed) {
    return (packed & ACOUSTIC_INFER_TFLM_AVAILABLE_BIT) != 0U;
}

#define ACOUSTIC_NAV_FLAG_RAW_DOA_VALID       (1U << 0)
#define ACOUSTIC_NAV_FLAG_FILTERED_DOA_VALID  (1U << 1)
#define ACOUSTIC_NAV_FLAG_SOURCE_VALID        (1U << 2)
#define ACOUSTIC_NAV_FLAG_GEOMETRY_VALID      (1U << 3)
#define ACOUSTIC_NAV_FLAG_TARGET_VALID        (1U << 4)
#define ACOUSTIC_NAV_FLAG_ARRIVAL_CANDIDATE   (1U << 5)
#define ACOUSTIC_NAV_FLAG_ARRIVED             (1U << 6)
#define ACOUSTIC_NAV_FLAG_MIN_TRANSLATION_TURN (1U << 7)

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
    ACOUSTIC_MESSAGE_NAV_DIAGNOSTICS = 0x23U,
    ACOUSTIC_MESSAGE_AI_LAB_SNAPSHOT = 0x30U,             /**< AIラボsnapshot */
    ACOUSTIC_MESSAGE_AI_LAB_SUMMARY_CHUNK = 0x31U,       /**< AIラボ要約chunk */
    ACOUSTIC_MESSAGE_AI_LAB_COMMAND = 0x32U,              /**< AIラボ操作要求 */
    ACOUSTIC_MESSAGE_AI_LAB_COMMAND_RESULT = 0x33U,      /**< AIラボ操作結果 */
    ACOUSTIC_MESSAGE_AI_LAB_PROFILE_CHUNK = 0x34U,        /**< AIラボ保存見本chunk */
    ACOUSTIC_MESSAGE_LOG = 0x7FU,
} acoustic_message_type_t;

/**< PC直結の音響AIラボからCPU0へ送る操作種別 */
typedef enum e_acoustic_ai_lab_command {
    ACOUSTIC_AI_LAB_COMMAND_STATUS = 0U,                    /**< 即時snapshot要求 */
    ACOUSTIC_AI_LAB_COMMAND_LEARNING_START = 1U,            /**< 新しい見本収集開始 */
    ACOUSTIC_AI_LAB_COMMAND_LEARNING_COMMIT = 2U,           /**< 5見本をMRAMへ保存 */
    ACOUSTIC_AI_LAB_COMMAND_LEARNING_CANCEL = 3U,           /**< 未保存収集を破棄 */
    ACOUSTIC_AI_LAB_COMMAND_PROFILE_READ = 4U,              /**< 保存済み5見本の読出し */
    ACOUSTIC_AI_LAB_COMMAND_RESTART = 5U,                   /**< 停止・待機状態から走行再開 */
} acoustic_ai_lab_command_t;

/**< AIラボ操作の受付結果 */
typedef enum e_acoustic_ai_lab_command_result {
    ACOUSTIC_AI_LAB_COMMAND_ACCEPTED = 0U,                  /**< CPU0思考タスクへ受付済み */
    ACOUSTIC_AI_LAB_COMMAND_REJECTED = 1U,                  /**< 状態または引数が不正 */
    ACOUSTIC_AI_LAB_COMMAND_UNAVAILABLE = 2U,               /**< 推論サービス未初期化 */
} acoustic_ai_lab_command_result_t;

typedef enum e_acoustic_xvf_status {
    ACOUSTIC_XVF_STATUS_STARTING = 0U,
    ACOUSTIC_XVF_STATUS_READY = 1U,
    ACOUSTIC_XVF_STATUS_ERROR = 2U,
} acoustic_xvf_status_t;

typedef struct st_acoustic_observation {
    uint16_t doa_deg;                                      /**< 循環平均後DoA[deg] */
    uint16_t raw_doa_deg;                                  /**< XVF3800直近DoA[deg] */
    int16_t level_dbfs_x100;
    int16_t peak_dbfs_x100;
    uint8_t vad;
    uint8_t doa_confidence;                                /**< 分散・取得経路から求めた品質[0..100] */
    uint8_t xvf_status;
    uint8_t audio_flags;
    uint8_t xvf_raw_status;
    uint8_t reserved;
    uint32_t audio_frame_count;
    uint32_t sample_sequence;                              /**< DoA観測専用sequence */
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
    uint8_t infer_status;
    uint8_t infer_sample_count;
    uint8_t infer_active_frames;
    uint8_t infer_nearest_sample;
    uint16_t infer_cosine_dist_x1000;
    uint16_t infer_threshold_x1000;
    uint32_t command_sequence;
    int32_t command_last_error;
    uint32_t command_target_age_ms;
    uint8_t infer_target_peak_bin;
    uint8_t infer_current_peak_bin;
    uint16_t infer_similarity_permille;
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

typedef struct st_acoustic_pose_telemetry {
    int32_t x_mm;
    int32_t y_mm;
    int32_t theta_mrad;
    int32_t v_mm_s;
    int32_t omega_mrad_s;
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    int16_t gyro_bias_dps_x10;
    uint8_t flags;
    uint8_t reserved;
    uint32_t uptime_ms;
} acoustic_pose_telemetry_t;

/**< 音源位置・到着・回避判断の診断情報。ROVER_TELEMETRYとは別フレームで送る。 */
typedef struct st_acoustic_nav_diagnostics {
    uint8_t schema_version;
    uint8_t flags;
    uint8_t doa_confidence;
    uint8_t arrival_state;
    uint32_t observation_sequence;
    uint16_t raw_doa_deg;
    uint16_t filtered_doa_deg;
    int32_t rover_x_mm;
    int32_t rover_y_mm;
    int32_t rover_heading_mrad;
    int32_t source_x_mm;
    int32_t source_y_mm;
    uint32_t source_range_mm;
    int16_t source_bearing_deg;
    uint8_t source_confidence;
    uint8_t observation_count;
    uint16_t localization_residual_mm;
    uint16_t crossing_angle_deg;
    uint16_t baseline_mm;
    uint16_t source_position_shift_mm;
    uint8_t arrival_confirm_count;
    uint8_t sensor_rule;
    uint8_t think_state;
    uint8_t reserved;
    uint32_t autonomous_backup_count;
} acoustic_nav_diagnostics_t;

/**< PC上の音響AIラボが継続記録する、CPU0推論の縮約snapshot */
typedef struct st_acoustic_ai_lab_snapshot {
    uint8_t schema_version;                                 /**< 通信schema版数 */
    uint8_t flags;                                          /**< 状態bit */
    uint8_t think_state;                                    /**< 音源追従状態 */
    uint8_t infer_status;                                   /**< 音響識別判定状態 */
    uint16_t doa_deg;                                       /**< 最新DoA */
    int16_t level_dbfs_x100;                                /**< 最新レベル */
    int16_t peak_dbfs_x100;                                 /**< 最新ピーク */
    uint8_t vad;                                            /**< XVF VAD */
    uint8_t xvf_status;                                     /**< XVF状態 */
    uint8_t audio_flags;                                    /**< フロントエンド異常bit */
    uint8_t learning_samples;                               /**< 現在収集済み見本数 */
    uint8_t target_peak_bin;                                /**< 保存見本の代表ピークbin */
    uint8_t current_peak_bin;                               /**< 今回特徴量のピークbin */
    uint8_t nearest_sample;                                 /**< 最近傍見本番号 */
    uint8_t active_frame_count;                             /**< 80 frame中の能動frame数 */
    uint16_t cosine_distance_x1000;                         /**< 最小cosine距離を1000倍した値 */
    uint16_t identifier_threshold_x1000;                    /**< 実効受理しきい値 */
    uint16_t similarity_permille;                           /**< 1000から距離を引いた類似度 */
    uint16_t background_mse_x1000;                          /**< 背景再構成MSE */
    uint16_t background_threshold_x1000;                    /**< 背景異常しきい値 */
    uint32_t observation_sequence;                          /**< 音響観測sequence */
    uint32_t feature_generation;                            /**< 最新80フレーム特徴量世代 */
    uint32_t inference_count;                               /**< 推論回数 */
    uint32_t match_count;                                   /**< TARGET累積回数 */
    uint8_t storage_result;                                 /**< MRAM直近結果 */
    uint8_t reserved[3];                                    /**< v2: 状態/連続強不一致数/最終TARGET経過時間10ms単位 */
} acoustic_ai_lab_snapshot_t;

/**< AI Lab 192次元要約の64-byte transport chunk */
typedef struct st_acoustic_ai_lab_summary_chunk {
    uint32_t feature_generation;
    uint8_t chunk_index;
    uint8_t chunk_count;
    uint8_t schema_version;
    uint8_t cpu_drop_count;
    int8_t data[ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE];
} acoustic_ai_lab_summary_chunk_t;

/**< AI Lab 保存見本の64-byte transport chunk */
typedef struct st_acoustic_ai_lab_profile_chunk {
    uint32_t profile_generation;
    uint8_t sample_index;
    uint8_t chunk_index;
    uint8_t chunk_count;
    uint8_t sample_count;
    int8_t data[ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE];
} acoustic_ai_lab_profile_chunk_t;

#define ACOUSTIC_AI_LAB_FLAG_LINK_READY          (1U << 0) /**< PC直結リンク確立済み */
#define ACOUSTIC_AI_LAB_FLAG_SUMMARY_VALID       (1U << 1) /**< 要約が有効 */
#define ACOUSTIC_AI_LAB_FLAG_BACKGROUND_ANOMALY  (1U << 2) /**< 背景異常を検出 */
#define ACOUSTIC_AI_LAB_FLAG_LEARNING_ACTIVE     (1U << 3) /**< 現場学習中 */
#define ACOUSTIC_AI_LAB_FLAG_STORAGE_VALID       (1U << 4) /**< 保存見本が有効 */

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
size_t acoustic_protocol_encode_pose_telemetry(uint32_t sequence, uint32_t uptime_ms,
                                               const acoustic_pose_telemetry_t * p_telemetry,
                                               uint8_t * p_output, size_t output_capacity);
size_t acoustic_protocol_encode_nav_diagnostics(uint32_t sequence, uint32_t uptime_ms,
                                                const acoustic_nav_diagnostics_t * p_diagnostics,
                                                uint8_t * p_output, size_t output_capacity);
size_t acoustic_protocol_encode_ai_lab_snapshot(uint32_t sequence, uint32_t uptime_ms,
                                                const acoustic_ai_lab_snapshot_t * p_snapshot,
                                                uint8_t * p_output, size_t output_capacity); /* AIラボsnapshot */
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
bool acoustic_protocol_decode_pose_telemetry(const acoustic_frame_t * p_frame,
                                             acoustic_pose_telemetry_t * p_telemetry);
bool acoustic_protocol_decode_nav_diagnostics(const acoustic_frame_t * p_frame,
                                              acoustic_nav_diagnostics_t * p_diagnostics);
bool acoustic_protocol_decode_ai_lab_snapshot(const acoustic_frame_t * p_frame,
                                              acoustic_ai_lab_snapshot_t * p_snapshot); /* AIラボsnapshot復号 */
bool acoustic_protocol_decode_ai_lab_summary_chunk(const acoustic_frame_t * p_frame,
                                                   acoustic_ai_lab_summary_chunk_t * p_chunk); /* 要約chunk復号 */
bool acoustic_protocol_decode_ai_lab_profile_chunk(const acoustic_frame_t * p_frame,
                                                   acoustic_ai_lab_profile_chunk_t * p_chunk); /* 見本chunk復号 */

#endif /* SEROV_ACOUSTIC_PROTOCOL_H */
