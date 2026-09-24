/** =================================================================*
 * @file   acoustic_protocol.c
 * @brief  ReSpeaker・RA8P1間音響通信プロトコル処理
 * ================================================================= */
#include "acoustic_protocol.h"                              /* 音響通信の型、定数、API */
#include <string.h>                                         /* memcpy、memset */

#define ACOUSTIC_FRAME_VERSION_OFFSET      (2U)
#define ACOUSTIC_FRAME_TYPE_OFFSET         (3U)
#define ACOUSTIC_FRAME_LENGTH_OFFSET       (4U)
#define ACOUSTIC_FRAME_SEQUENCE_OFFSET     (6U)
#define ACOUSTIC_FRAME_UPTIME_OFFSET       (10U)
#define ACOUSTIC_FRAME_PAYLOAD_OFFSET      ACOUSTIC_PROTOCOL_HEADER_SIZE

static void acoustic_write_u16_le(uint8_t * p_output, uint16_t value); /* 16 bit LE書込 */
static void acoustic_write_u32_le(uint8_t * p_output, uint32_t value); /* 32 bit LE書込 */
static uint16_t acoustic_read_u16_le(const uint8_t * p_input); /* 16 bit LE読出 */
static uint32_t acoustic_read_u32_le(const uint8_t * p_input); /* 32 bit LE読出 */
static void acoustic_protocol_parser_resync(acoustic_protocol_parser_t * p_parser, uint8_t byte); /* magic再同期 */

/** =================================================================*
 * @brief  16 bit little-endian書込
 * @param[out] p_output 2 byte以上の出力先
 * @param[in] value 書き込む値
 * ================================================================= */
static void acoustic_write_u16_le(uint8_t * p_output, uint16_t value) {
    p_output[0] = (uint8_t) value;
    p_output[1] = (uint8_t) (value >> 8U);
}

/** =================================================================*
 * @brief  32 bit little-endian書込
 * @param[out] p_output 4 byte以上の出力先
 * @param[in] value 書き込む値
 * ================================================================= */
static void acoustic_write_u32_le(uint8_t * p_output, uint32_t value) {
    p_output[0] = (uint8_t) value;
    p_output[1] = (uint8_t) (value >> 8U);
    p_output[2] = (uint8_t) (value >> 16U);
    p_output[3] = (uint8_t) (value >> 24U);
}

/** =================================================================*
 * @brief  16 bit little-endian読出
 * @param[in] p_input 2 byte以上の入力元
 * @return 読み出した値
 * ================================================================= */
static uint16_t acoustic_read_u16_le(const uint8_t * p_input) {
    return (uint16_t) ((uint16_t) p_input[0] | ((uint16_t) p_input[1] << 8U));
}

/** =================================================================*
 * @brief  32 bit little-endian読出
 * @param[in] p_input 4 byte以上の入力元
 * @return 読み出した値
 * ================================================================= */
static uint32_t acoustic_read_u32_le(const uint8_t * p_input) {
    return (uint32_t) p_input[0] | ((uint32_t) p_input[1] << 8U) | ((uint32_t) p_input[2] << 16U) |
           ((uint32_t) p_input[3] << 24U);
}

/** =================================================================*
 * @brief  CRC-16/CCITT-FALSE計算
 * @param[in] p_data 入力データ
 * @param[in] length 入力長
 * @return CRC値
 * ================================================================= */
uint16_t acoustic_protocol_crc16(const uint8_t * p_data, size_t length) {
    uint16_t crc = 0xFFFFU;

    if ((NULL == p_data) && (0U != length)) {
        return 0U;
    }

    for (size_t index = 0U; index < length; index++) {
        crc ^= (uint16_t) p_data[index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (0U != (crc & 0x8000U)) ? (uint16_t) ((crc << 1U) ^ 0x1021U) : (uint16_t) (crc << 1U);
        }
    }

    return crc;
}

/** =================================================================*
 * @brief  汎用フレーム符号化
 * @details payloadは呼出側でプロトコルのlittle-endian表現に変換する。
 * @param[in] type メッセージ種別
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms ESP32起動後時間
 * @param[in] p_payload ワイヤ形式ペイロード
 * @param[in] payload_length ペイロード長
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode(acoustic_message_type_t type, uint32_t sequence, uint32_t uptime_ms,
                                const uint8_t * p_payload, uint16_t payload_length, uint8_t * p_output,
                                size_t output_capacity) {
    size_t const frame_size = ACOUSTIC_PROTOCOL_HEADER_SIZE + (size_t) payload_length + ACOUSTIC_PROTOCOL_CRC_SIZE;

    if ((NULL == p_output) || ((NULL == p_payload) && (0U != payload_length)) || (0U == (uint8_t) type) ||
        (payload_length > ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE) || (output_capacity < frame_size)) {
        return 0U;
    }

    p_output[0] = ACOUSTIC_PROTOCOL_MAGIC_0;
    p_output[1] = ACOUSTIC_PROTOCOL_MAGIC_1;
    p_output[ACOUSTIC_FRAME_VERSION_OFFSET] = ACOUSTIC_PROTOCOL_VERSION;
    p_output[ACOUSTIC_FRAME_TYPE_OFFSET] = (uint8_t) type;
    acoustic_write_u16_le(&p_output[ACOUSTIC_FRAME_LENGTH_OFFSET], payload_length);
    acoustic_write_u32_le(&p_output[ACOUSTIC_FRAME_SEQUENCE_OFFSET], sequence);
    acoustic_write_u32_le(&p_output[ACOUSTIC_FRAME_UPTIME_OFFSET], uptime_ms);
    if (0U != payload_length) {
        memcpy(&p_output[ACOUSTIC_FRAME_PAYLOAD_OFFSET], p_payload, payload_length);
    }

    uint16_t const crc =
        acoustic_protocol_crc16(&p_output[ACOUSTIC_FRAME_VERSION_OFFSET],
                                frame_size - ACOUSTIC_FRAME_VERSION_OFFSET - ACOUSTIC_PROTOCOL_CRC_SIZE);
    acoustic_write_u16_le(&p_output[frame_size - ACOUSTIC_PROTOCOL_CRC_SIZE], crc);
    return frame_size;
}

/** =================================================================*
 * @brief  音響観測符号化
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms ESP32起動後時間
 * @param[in] p_observation 音響観測
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_observation(uint32_t sequence, uint32_t uptime_ms,
                                            const acoustic_observation_t * p_observation, uint8_t * p_output,
                                            size_t output_capacity) {
    if (NULL == p_observation) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_OBSERVATION_PAYLOAD_SIZE];
    acoustic_write_u16_le(&payload[0], p_observation->doa_deg);
    acoustic_write_u16_le(&payload[2], p_observation->raw_doa_deg);
    acoustic_write_u16_le(&payload[4], (uint16_t) p_observation->level_dbfs_x100);
    acoustic_write_u16_le(&payload[6], (uint16_t) p_observation->peak_dbfs_x100);
    payload[8] = p_observation->vad;
    payload[9] = p_observation->doa_confidence;
    payload[10] = p_observation->xvf_status;
    payload[11] = p_observation->audio_flags;
    payload[12] = p_observation->xvf_raw_status;
    payload[13] = p_observation->reserved;
    acoustic_write_u32_le(&payload[14], p_observation->audio_frame_count);
    acoustic_write_u32_le(&payload[18], p_observation->sample_sequence);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_OBSERVATION, sequence, uptime_ms, payload,
                                    (uint16_t) sizeof(payload), p_output, output_capacity);
}

/** =================================================================*
 * @brief  起動情報符号化
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms ESP32起動後時間
 * @param[in] p_hello 起動情報
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_hello(uint32_t sequence, uint32_t uptime_ms, const acoustic_hello_t * p_hello,
                                      uint8_t * p_output, size_t output_capacity) {
    if (NULL == p_hello) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_HELLO_PAYLOAD_SIZE];
    payload[0] = p_hello->firmware_major;
    payload[1] = p_hello->firmware_minor;
    payload[2] = p_hello->firmware_patch;
    payload[3] = p_hello->reserved;
    acoustic_write_u32_le(&payload[4], p_hello->capabilities);
    acoustic_write_u32_le(&payload[8], p_hello->boot_id);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_HELLO, sequence, uptime_ms, payload, (uint16_t) sizeof(payload),
                                    p_output, output_capacity);
}

/** =================================================================*
 * @brief  健全性情報符号化
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms ESP32起動後時間
 * @param[in] p_health 健全性情報
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_health(uint32_t sequence, uint32_t uptime_ms, const acoustic_health_t * p_health,
                                       uint8_t * p_output, size_t output_capacity) {
    if (NULL == p_health) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_HEALTH_PAYLOAD_SIZE];
    payload[0] = p_health->xvf_status;
    payload[1] = p_health->audio_flags;
    payload[2] = p_health->usb_connected;
    payload[3] = p_health->wifi_connected;
    acoustic_write_u32_le(&payload[4], p_health->i2c_error_count);
    acoustic_write_u32_le(&payload[8], p_health->i2s_overrun_count);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_HEALTH, sequence, uptime_ms, payload, (uint16_t) sizeof(payload),
                                    p_output, output_capacity);
}

/** =================================================================*
 * @brief  log-mel特徴量符号化
 * @details メタデータ8 byteと2フレーム分のint8 melを固定72 byteで送る。
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms ESP32起動後時間
 * @param[in] p_feature log-mel特徴量
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_feature(uint32_t sequence, uint32_t uptime_ms,
                                        const acoustic_feature_t * p_feature, uint8_t * p_output,
                                        size_t output_capacity) {
    if (NULL == p_feature) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_FEATURE_PAYLOAD_SIZE];
    acoustic_write_u16_le(&payload[0], p_feature->event_id);
    acoustic_write_u16_le(&payload[2], p_feature->frame_index);
    acoustic_write_u16_le(&payload[4], p_feature->frame_count);
    payload[6] = p_feature->n_bins;
    payload[7] = p_feature->flags;
    memcpy(&payload[ACOUSTIC_FEATURE_META_SIZE], p_feature->mel, ACOUSTIC_FEATURE_DATA_SIZE);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_FEATURE, sequence, uptime_ms, payload,
                                    (uint16_t) sizeof(payload), p_output, output_capacity);
}

/** =================================================================*
 * @brief  ローバ診断情報符号化
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms CPU0起動後時間
 * @param[in] p_telemetry ローバ診断情報
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_rover_telemetry(uint32_t sequence, uint32_t uptime_ms,
                                                const acoustic_rover_telemetry_t * p_telemetry, uint8_t * p_output,
                                                size_t output_capacity) {
    if (NULL == p_telemetry) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_ROVER_TELEMETRY_PAYLOAD_SIZE];
    payload[0] = p_telemetry->schema_version;
    payload[1] = p_telemetry->think_state;
    payload[2] = p_telemetry->usb_state;
    payload[3] = p_telemetry->flags;
    acoustic_write_u16_le(&payload[4], p_telemetry->doa_deg);
    acoustic_write_u16_le(&payload[6], (uint16_t) p_telemetry->level_dbfs_x100);
    acoustic_write_u16_le(&payload[8], (uint16_t) p_telemetry->peak_dbfs_x100);
    payload[10] = p_telemetry->vad;
    payload[11] = p_telemetry->xvf_status;
    payload[12] = p_telemetry->audio_flags;
    payload[13] = p_telemetry->xvf_raw_status;
    acoustic_write_u32_le(&payload[14], p_telemetry->observation_sequence);
    acoustic_write_u32_le(&payload[18], p_telemetry->observation_age_ms);
    acoustic_write_u32_le(&payload[22], p_telemetry->audio_frame_count);
    acoustic_write_u32_le(&payload[26], p_telemetry->audio_crc_error_count);
    acoustic_write_u32_le(&payload[30], p_telemetry->fault_flags);
    acoustic_write_u16_le(&payload[34], (uint16_t) p_telemetry->steering_deg);
    acoustic_write_u16_le(&payload[36], (uint16_t) p_telemetry->left_target_rpm);
    acoustic_write_u16_le(&payload[38], (uint16_t) p_telemetry->right_target_rpm);
    payload[40] = p_telemetry->infer_status;
    payload[41] = p_telemetry->infer_sample_count;
    payload[42] = p_telemetry->infer_active_frames;
    payload[43] = p_telemetry->infer_nearest_sample;
    acoustic_write_u16_le(&payload[44], p_telemetry->infer_cosine_dist_x1000);
    acoustic_write_u16_le(&payload[46], p_telemetry->infer_threshold_x1000);
    acoustic_write_u32_le(&payload[48], p_telemetry->command_sequence);
    acoustic_write_u32_le(&payload[52], (uint32_t) p_telemetry->command_last_error);
    acoustic_write_u32_le(&payload[56], p_telemetry->command_target_age_ms);
    payload[60] = p_telemetry->infer_target_peak_bin;
    payload[61] = p_telemetry->infer_current_peak_bin;
    acoustic_write_u16_le(&payload[62], p_telemetry->infer_similarity_permille);
    payload[64] = p_telemetry->autonomy_mode;
    payload[65] = p_telemetry->sensor_rule;
    payload[66] = p_telemetry->sensor_valid_flags;
    payload[67] = p_telemetry->sensor_reserved;
    for (uint32_t index = 0U; index < 3U; index++) {
        acoustic_write_u16_le(&payload[68U + (index * 2U)], p_telemetry->tof_distance_mm[index]);
        acoustic_write_u16_le(&payload[74U + (index * 2U)], (uint16_t) p_telemetry->accel_mg[index]);
        acoustic_write_u16_le(&payload[80U + (index * 2U)], (uint16_t) p_telemetry->gyro_dps_x10[index]);
    }
    acoustic_write_u16_le(&payload[86], p_telemetry->sensor_error_flags);
    acoustic_write_u32_le(&payload[88], (uint32_t) p_telemetry->sensor_last_error);
    acoustic_write_u32_le(&payload[92], p_telemetry->sensor_age_ms);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_ROVER_TELEMETRY, sequence, uptime_ms, payload,
                                    (uint16_t) sizeof(payload), p_output, output_capacity);
}

/** =================================================================*
 * @brief  CPU1アクチュエータ診断情報符号化
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms CPU0起動後時間
 * @param[in] p_telemetry CPU1アクチュエータ診断情報
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_actuator_telemetry(uint32_t sequence, uint32_t uptime_ms,
                                                   const acoustic_actuator_telemetry_t * p_telemetry,
                                                   uint8_t * p_output, size_t output_capacity) {
    if (NULL == p_telemetry) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_ACTUATOR_TELEMETRY_PAYLOAD_SIZE];
    payload[0] = p_telemetry->schema_version;
    payload[1] = p_telemetry->status_valid;
    acoustic_write_u16_le(&payload[2], p_telemetry->fault_flags);
    acoustic_write_u32_le(&payload[4], p_telemetry->actuator_status_age_ms);
    acoustic_write_u32_le(&payload[8], p_telemetry->actuator_status_sequence);
    acoustic_write_u32_le(&payload[12], p_telemetry->actuator_applied_command_sequence);
    acoustic_write_u16_le(&payload[16], (uint16_t) p_telemetry->actuator_left_duty_permille);
    acoustic_write_u16_le(&payload[18], (uint16_t) p_telemetry->actuator_right_duty_permille);
    acoustic_write_u16_le(&payload[20], (uint16_t) p_telemetry->actuator_left_encoder_rpm_x10);
    acoustic_write_u16_le(&payload[22], (uint16_t) p_telemetry->actuator_right_encoder_rpm_x10);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_ACTUATOR_TELEMETRY, sequence, uptime_ms, payload,
                                    (uint16_t) sizeof(payload), p_output, output_capacity);
}

/** =================================================================*
 * @brief  オドメトリ位置姿勢テレメトリ符号化
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms CPU0起動後時間
 * @param[in] p_telemetry オドメトリ位置姿勢テレメトリ
 * @param[out] p_output フレーム出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_pose_telemetry(uint32_t sequence, uint32_t uptime_ms,
                                               const acoustic_pose_telemetry_t * p_telemetry,
                                               uint8_t * p_output, size_t output_capacity) {
    if (NULL == p_telemetry) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_POSE_TELEMETRY_PAYLOAD_SIZE];
    acoustic_write_u32_le(&payload[0], (uint32_t) p_telemetry->x_mm);
    acoustic_write_u32_le(&payload[4], (uint32_t) p_telemetry->y_mm);
    acoustic_write_u32_le(&payload[8], (uint32_t) p_telemetry->theta_mrad);
    acoustic_write_u32_le(&payload[12], (uint32_t) p_telemetry->v_mm_s);
    acoustic_write_u32_le(&payload[16], (uint32_t) p_telemetry->omega_mrad_s);
    acoustic_write_u32_le(&payload[20], (uint32_t) p_telemetry->left_encoder_count);
    acoustic_write_u32_le(&payload[24], (uint32_t) p_telemetry->right_encoder_count);
    acoustic_write_u16_le(&payload[28], (uint16_t) p_telemetry->gyro_bias_dps_x10);
    payload[30] = p_telemetry->flags;
    payload[31] = p_telemetry->reserved;
    acoustic_write_u32_le(&payload[32], p_telemetry->uptime_ms);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_POSE_TELEMETRY, sequence, uptime_ms, payload,
                                    (uint16_t) sizeof(payload), p_output, output_capacity);
}

/** =================================================================*
 * @brief  音源位置・到着・回避診断情報を符号化
 * @param[in] sequence 送信シーケンス
 * @param[in] uptime_ms CPU0起動後時間
 * @param[in] p_diagnostics 音源ナビゲーション診断
 * @param[out] p_output 出力先
 * @param[in] output_capacity 出力先容量
 * @return 符号化長。引数不正時は0
 * ================================================================= */
size_t acoustic_protocol_encode_nav_diagnostics(uint32_t sequence, uint32_t uptime_ms,
                                                const acoustic_nav_diagnostics_t * p_diagnostics,
                                                uint8_t * p_output, size_t output_capacity) {
    if (NULL == p_diagnostics) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_NAV_DIAGNOSTICS_PAYLOAD_SIZE] = {0};
    payload[0] = p_diagnostics->schema_version;
    payload[1] = p_diagnostics->flags;
    payload[2] = p_diagnostics->doa_confidence;
    payload[3] = p_diagnostics->arrival_state;
    acoustic_write_u32_le(&payload[4], p_diagnostics->observation_sequence);
    acoustic_write_u16_le(&payload[8], p_diagnostics->raw_doa_deg);
    acoustic_write_u16_le(&payload[10], p_diagnostics->filtered_doa_deg);
    acoustic_write_u32_le(&payload[12], (uint32_t) p_diagnostics->rover_x_mm);
    acoustic_write_u32_le(&payload[16], (uint32_t) p_diagnostics->rover_y_mm);
    acoustic_write_u32_le(&payload[20], (uint32_t) p_diagnostics->rover_heading_mrad);
    acoustic_write_u32_le(&payload[24], (uint32_t) p_diagnostics->source_x_mm);
    acoustic_write_u32_le(&payload[28], (uint32_t) p_diagnostics->source_y_mm);
    acoustic_write_u32_le(&payload[32], p_diagnostics->source_range_mm);
    acoustic_write_u16_le(&payload[36], (uint16_t) p_diagnostics->source_bearing_deg);
    payload[38] = p_diagnostics->source_confidence;
    payload[39] = p_diagnostics->observation_count;
    acoustic_write_u16_le(&payload[40], p_diagnostics->localization_residual_mm);
    acoustic_write_u16_le(&payload[42], p_diagnostics->crossing_angle_deg);
    acoustic_write_u16_le(&payload[44], p_diagnostics->baseline_mm);
    acoustic_write_u16_le(&payload[46], p_diagnostics->source_position_shift_mm);
    payload[48] = p_diagnostics->arrival_confirm_count;
    payload[49] = p_diagnostics->sensor_rule;
    payload[50] = p_diagnostics->think_state;
    payload[51] = p_diagnostics->reserved;
    acoustic_write_u32_le(&payload[52], p_diagnostics->autonomous_backup_count);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_NAV_DIAGNOSTICS, sequence, uptime_ms, payload,
                                    (uint16_t) sizeof(payload), p_output, output_capacity);
}

/** =================================================================*
 * @brief  PC直結音響AIラボ用の推論snapshotを符号化
 * @details すべての浮動小数値は固定小数点へ縮約し、MCU/PC間で同一の
 *          診断値を扱う。特徴量本体は別のchunk frameで送る。
 * @param[in] sequence 送信sequence
 * @param[in] uptime_ms 送信元の稼働時間[ms]
 * @param[in] p_snapshot 符号化するsnapshot
 * @param[out] p_output 出力バッファ
 * @param[in] output_capacity 出力バッファ容量[byte]
 * @return 符号化したフレーム長[byte]、失敗時は0
 * ================================================================= */
size_t acoustic_protocol_encode_ai_lab_snapshot(uint32_t sequence, uint32_t uptime_ms,
                                                const acoustic_ai_lab_snapshot_t * p_snapshot,
                                                uint8_t * p_output, size_t output_capacity) {
    if (NULL == p_snapshot) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_AI_LAB_SNAPSHOT_PAYLOAD_SIZE] = {0};
    payload[0] = p_snapshot->schema_version;
    payload[1] = p_snapshot->flags;
    payload[2] = p_snapshot->think_state;
    payload[3] = p_snapshot->infer_status;
    acoustic_write_u16_le(&payload[4], p_snapshot->doa_deg);
    acoustic_write_u16_le(&payload[6], (uint16_t) p_snapshot->level_dbfs_x100);
    acoustic_write_u16_le(&payload[8], (uint16_t) p_snapshot->peak_dbfs_x100);
    payload[10] = p_snapshot->vad;
    payload[11] = p_snapshot->xvf_status;
    payload[12] = p_snapshot->audio_flags;
    payload[13] = p_snapshot->learning_samples;
    payload[14] = p_snapshot->target_peak_bin;
    payload[15] = p_snapshot->current_peak_bin;
    payload[16] = p_snapshot->nearest_sample;
    payload[17] = p_snapshot->active_frame_count;
    acoustic_write_u16_le(&payload[18], p_snapshot->cosine_distance_x1000);
    acoustic_write_u16_le(&payload[20], p_snapshot->identifier_threshold_x1000);
    acoustic_write_u16_le(&payload[22], p_snapshot->similarity_permille);
    acoustic_write_u16_le(&payload[24], p_snapshot->background_mse_x1000);
    acoustic_write_u16_le(&payload[26], p_snapshot->background_threshold_x1000);
    acoustic_write_u32_le(&payload[28], p_snapshot->observation_sequence);
    acoustic_write_u32_le(&payload[32], p_snapshot->feature_generation);
    acoustic_write_u32_le(&payload[36], p_snapshot->inference_count);
    acoustic_write_u32_le(&payload[40], p_snapshot->match_count);
    payload[44] = p_snapshot->storage_result;
    payload[45] = p_snapshot->reserved[0];
    payload[46] = p_snapshot->reserved[1];
    payload[47] = p_snapshot->reserved[2];

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_AI_LAB_SNAPSHOT, sequence, uptime_ms, payload,
                                    (uint16_t) sizeof(payload), p_output, output_capacity);
}

/** =================================================================*
 * @brief  パーサー初期化
 * @param[out] p_parser パーサー状態
 * ================================================================= */
void acoustic_protocol_parser_init(acoustic_protocol_parser_t * p_parser) {
    if (NULL != p_parser) {
        memset(p_parser, 0, sizeof(*p_parser));
    }
}

/** =================================================================*
 * @brief  magic再同期
 * @param[in,out] p_parser パーサー状態
 * @param[in] byte 現在の入力byte
 * ================================================================= */
static void acoustic_protocol_parser_resync(acoustic_protocol_parser_t * p_parser, uint8_t byte) {
    p_parser->used = 0U;
    p_parser->expected = 0U;
    if (ACOUSTIC_PROTOCOL_MAGIC_0 == byte) {
        p_parser->bytes[0] = byte;
        p_parser->used = 1U;
    }
}

/** =================================================================*
 * @brief  1 byte受信
 * @details USB転送境界に依存せず、magic・長さ・CRCでフレームを復元する。
 * @param[in,out] p_parser パーサー状態
 * @param[in] byte 受信byte
 * @param[out] p_frame 完成フレーム
 * @return パース結果
 * ================================================================= */
acoustic_parse_result_t acoustic_protocol_parser_push(acoustic_protocol_parser_t * p_parser, uint8_t byte,
                                                      acoustic_frame_t * p_frame) {
    if ((NULL == p_parser) || (NULL == p_frame)) {
        return ACOUSTIC_PARSE_FORMAT_ERROR;
    }

    if (0U == p_parser->used) {
        if (ACOUSTIC_PROTOCOL_MAGIC_0 == byte) {
            p_parser->bytes[0] = byte;
            p_parser->used = 1U;
        }
        return ACOUSTIC_PARSE_MORE;
    }

    if (1U == p_parser->used) {
        if (ACOUSTIC_PROTOCOL_MAGIC_1 == byte) {
            p_parser->bytes[1] = byte;
            p_parser->used = 2U;
        } else {
            acoustic_protocol_parser_resync(p_parser, byte);
        }
        return ACOUSTIC_PARSE_MORE;
    }

    if (p_parser->used >= ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE) {
        acoustic_protocol_parser_resync(p_parser, byte);
        return ACOUSTIC_PARSE_FORMAT_ERROR;
    }

    p_parser->bytes[p_parser->used++] = byte;
    if (p_parser->used == ACOUSTIC_PROTOCOL_HEADER_SIZE) {
        uint16_t const payload_length = acoustic_read_u16_le(&p_parser->bytes[ACOUSTIC_FRAME_LENGTH_OFFSET]);
        if (payload_length > ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE) {
            acoustic_protocol_parser_resync(p_parser, byte);
            return ACOUSTIC_PARSE_FORMAT_ERROR;
        }
        if (ACOUSTIC_PROTOCOL_VERSION != p_parser->bytes[ACOUSTIC_FRAME_VERSION_OFFSET]) {
            acoustic_protocol_parser_resync(p_parser, byte);
            return ACOUSTIC_PARSE_UNSUPPORTED_VERSION;
        }
        p_parser->expected = (uint16_t) (ACOUSTIC_PROTOCOL_HEADER_SIZE + payload_length + ACOUSTIC_PROTOCOL_CRC_SIZE);
    }

    if ((0U == p_parser->expected) || (p_parser->used < p_parser->expected)) {
        return ACOUSTIC_PARSE_MORE;
    }

    uint16_t const received_crc =
        acoustic_read_u16_le(&p_parser->bytes[p_parser->expected - ACOUSTIC_PROTOCOL_CRC_SIZE]);
    uint16_t const calculated_crc =
        acoustic_protocol_crc16(&p_parser->bytes[ACOUSTIC_FRAME_VERSION_OFFSET],
                                p_parser->expected - ACOUSTIC_FRAME_VERSION_OFFSET - ACOUSTIC_PROTOCOL_CRC_SIZE);
    if (received_crc != calculated_crc) {
        acoustic_protocol_parser_resync(p_parser, byte);
        return ACOUSTIC_PARSE_CRC_ERROR;
    }

    p_frame->version = p_parser->bytes[ACOUSTIC_FRAME_VERSION_OFFSET];
    p_frame->type = (acoustic_message_type_t) p_parser->bytes[ACOUSTIC_FRAME_TYPE_OFFSET];
    p_frame->payload_length = acoustic_read_u16_le(&p_parser->bytes[ACOUSTIC_FRAME_LENGTH_OFFSET]);
    p_frame->sequence = acoustic_read_u32_le(&p_parser->bytes[ACOUSTIC_FRAME_SEQUENCE_OFFSET]);
    p_frame->uptime_ms = acoustic_read_u32_le(&p_parser->bytes[ACOUSTIC_FRAME_UPTIME_OFFSET]);
    if (0U != p_frame->payload_length) {
        memcpy(p_frame->payload, &p_parser->bytes[ACOUSTIC_FRAME_PAYLOAD_OFFSET], p_frame->payload_length);
    }
    p_parser->used = 0U;
    p_parser->expected = 0U;
    return ACOUSTIC_PARSE_FRAME_READY;
}

/** =================================================================*
 * @brief  音響観測復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_observation 音響観測
 * @return フレームが正しい音響観測ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_observation(const acoustic_frame_t * p_frame, acoustic_observation_t * p_observation) {
    if ((NULL == p_frame) || (NULL == p_observation) || (ACOUSTIC_MESSAGE_OBSERVATION != p_frame->type) ||
        (ACOUSTIC_OBSERVATION_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_observation->doa_deg = acoustic_read_u16_le(&p_frame->payload[0]);
    p_observation->raw_doa_deg = acoustic_read_u16_le(&p_frame->payload[2]);
    p_observation->level_dbfs_x100 = (int16_t) acoustic_read_u16_le(&p_frame->payload[4]);
    p_observation->peak_dbfs_x100 = (int16_t) acoustic_read_u16_le(&p_frame->payload[6]);
    p_observation->vad = p_frame->payload[8];
    p_observation->doa_confidence = p_frame->payload[9];
    p_observation->xvf_status = p_frame->payload[10];
    p_observation->audio_flags = p_frame->payload[11];
    p_observation->xvf_raw_status = p_frame->payload[12];
    p_observation->reserved = p_frame->payload[13];
    p_observation->audio_frame_count = acoustic_read_u32_le(&p_frame->payload[14]);
    p_observation->sample_sequence = acoustic_read_u32_le(&p_frame->payload[18]);
    return true;
}

/** =================================================================*
 * @brief  起動情報復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_hello 起動情報
 * @return フレームが正しい起動情報ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_hello(const acoustic_frame_t * p_frame, acoustic_hello_t * p_hello) {
    if ((NULL == p_frame) || (NULL == p_hello) || (ACOUSTIC_MESSAGE_HELLO != p_frame->type) ||
        (ACOUSTIC_HELLO_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_hello->firmware_major = p_frame->payload[0];
    p_hello->firmware_minor = p_frame->payload[1];
    p_hello->firmware_patch = p_frame->payload[2];
    p_hello->reserved = p_frame->payload[3];
    p_hello->capabilities = acoustic_read_u32_le(&p_frame->payload[4]);
    p_hello->boot_id = acoustic_read_u32_le(&p_frame->payload[8]);
    return true;
}

/** =================================================================*
 * @brief  健全性情報復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_health 健全性情報
 * @return フレームが正しい健全性情報ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_health(const acoustic_frame_t * p_frame, acoustic_health_t * p_health) {
    if ((NULL == p_frame) || (NULL == p_health) || (ACOUSTIC_MESSAGE_HEALTH != p_frame->type) ||
        (ACOUSTIC_HEALTH_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_health->xvf_status = p_frame->payload[0];
    p_health->audio_flags = p_frame->payload[1];
    p_health->usb_connected = p_frame->payload[2];
    p_health->wifi_connected = p_frame->payload[3];
    p_health->i2c_error_count = acoustic_read_u32_le(&p_frame->payload[4]);
    p_health->i2s_overrun_count = acoustic_read_u32_le(&p_frame->payload[8]);
    return true;
}

/** =================================================================*
 * @brief  log-mel特徴量復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_feature log-mel特徴量
 * @return フレームが正しい特徴量メッセージならtrue
 * ================================================================= */
bool acoustic_protocol_decode_feature(const acoustic_frame_t * p_frame, acoustic_feature_t * p_feature) {
    if ((NULL == p_frame) || (NULL == p_feature) || (ACOUSTIC_MESSAGE_FEATURE != p_frame->type) ||
        (ACOUSTIC_FEATURE_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_feature->event_id = acoustic_read_u16_le(&p_frame->payload[0]);
    p_feature->frame_index = acoustic_read_u16_le(&p_frame->payload[2]);
    p_feature->frame_count = acoustic_read_u16_le(&p_frame->payload[4]);
    p_feature->n_bins = p_frame->payload[6];
    p_feature->flags = p_frame->payload[7];
    memcpy(p_feature->mel, &p_frame->payload[ACOUSTIC_FEATURE_META_SIZE], ACOUSTIC_FEATURE_DATA_SIZE);
    return true;
}

/** =================================================================*
 * @brief  ローバ診断情報復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_telemetry ローバ診断情報
 * @return フレームが正しいローバ診断情報ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_rover_telemetry(const acoustic_frame_t * p_frame,
                                              acoustic_rover_telemetry_t * p_telemetry) {
    if ((NULL == p_frame) || (NULL == p_telemetry) || (ACOUSTIC_MESSAGE_ROVER_TELEMETRY != p_frame->type) ||
        (ACOUSTIC_ROVER_TELEMETRY_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_telemetry->schema_version = p_frame->payload[0];
    p_telemetry->think_state = p_frame->payload[1];
    p_telemetry->usb_state = p_frame->payload[2];
    p_telemetry->flags = p_frame->payload[3];
    p_telemetry->doa_deg = acoustic_read_u16_le(&p_frame->payload[4]);
    p_telemetry->level_dbfs_x100 = (int16_t) acoustic_read_u16_le(&p_frame->payload[6]);
    p_telemetry->peak_dbfs_x100 = (int16_t) acoustic_read_u16_le(&p_frame->payload[8]);
    p_telemetry->vad = p_frame->payload[10];
    p_telemetry->xvf_status = p_frame->payload[11];
    p_telemetry->audio_flags = p_frame->payload[12];
    p_telemetry->xvf_raw_status = p_frame->payload[13];
    p_telemetry->observation_sequence = acoustic_read_u32_le(&p_frame->payload[14]);
    p_telemetry->observation_age_ms = acoustic_read_u32_le(&p_frame->payload[18]);
    p_telemetry->audio_frame_count = acoustic_read_u32_le(&p_frame->payload[22]);
    p_telemetry->audio_crc_error_count = acoustic_read_u32_le(&p_frame->payload[26]);
    p_telemetry->fault_flags = acoustic_read_u32_le(&p_frame->payload[30]);
    p_telemetry->steering_deg = (int16_t) acoustic_read_u16_le(&p_frame->payload[34]);
    p_telemetry->left_target_rpm = (int16_t) acoustic_read_u16_le(&p_frame->payload[36]);
    p_telemetry->right_target_rpm = (int16_t) acoustic_read_u16_le(&p_frame->payload[38]);
    p_telemetry->infer_status = p_frame->payload[40];
    p_telemetry->infer_sample_count = p_frame->payload[41];
    p_telemetry->infer_active_frames = p_frame->payload[42];
    p_telemetry->infer_nearest_sample = p_frame->payload[43];
    p_telemetry->infer_cosine_dist_x1000 = acoustic_read_u16_le(&p_frame->payload[44]);
    p_telemetry->infer_threshold_x1000 = acoustic_read_u16_le(&p_frame->payload[46]);
    p_telemetry->command_sequence = acoustic_read_u32_le(&p_frame->payload[48]);
    p_telemetry->command_last_error = (int32_t) acoustic_read_u32_le(&p_frame->payload[52]);
    p_telemetry->command_target_age_ms = acoustic_read_u32_le(&p_frame->payload[56]);
    p_telemetry->infer_target_peak_bin = p_frame->payload[60];
    p_telemetry->infer_current_peak_bin = p_frame->payload[61];
    p_telemetry->infer_similarity_permille = acoustic_read_u16_le(&p_frame->payload[62]);
    p_telemetry->autonomy_mode = p_frame->payload[64];
    p_telemetry->sensor_rule = p_frame->payload[65];
    p_telemetry->sensor_valid_flags = p_frame->payload[66];
    p_telemetry->sensor_reserved = p_frame->payload[67];
    for (uint32_t index = 0U; index < 3U; index++) {
        p_telemetry->tof_distance_mm[index] = acoustic_read_u16_le(&p_frame->payload[68U + (index * 2U)]);
        p_telemetry->accel_mg[index] = (int16_t) acoustic_read_u16_le(&p_frame->payload[74U + (index * 2U)]);
        p_telemetry->gyro_dps_x10[index] = (int16_t) acoustic_read_u16_le(&p_frame->payload[80U + (index * 2U)]);
    }
    p_telemetry->sensor_error_flags = acoustic_read_u16_le(&p_frame->payload[86]);
    p_telemetry->sensor_last_error = (int32_t) acoustic_read_u32_le(&p_frame->payload[88]);
    p_telemetry->sensor_age_ms = acoustic_read_u32_le(&p_frame->payload[92]);
    return true;
}

/** =================================================================*
 * @brief  CPU1アクチュエータ診断情報復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_telemetry CPU1アクチュエータ診断情報
 * @return フレームが正しいアクチュエータ診断情報ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_actuator_telemetry(const acoustic_frame_t * p_frame,
                                                 acoustic_actuator_telemetry_t * p_telemetry) {
    if ((NULL == p_frame) || (NULL == p_telemetry) || (ACOUSTIC_MESSAGE_ACTUATOR_TELEMETRY != p_frame->type) ||
        (ACOUSTIC_ACTUATOR_TELEMETRY_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_telemetry->schema_version = p_frame->payload[0];
    p_telemetry->status_valid = p_frame->payload[1];
    p_telemetry->fault_flags = acoustic_read_u16_le(&p_frame->payload[2]);
    p_telemetry->actuator_status_age_ms = acoustic_read_u32_le(&p_frame->payload[4]);
    p_telemetry->actuator_status_sequence = acoustic_read_u32_le(&p_frame->payload[8]);
    p_telemetry->actuator_applied_command_sequence = acoustic_read_u32_le(&p_frame->payload[12]);
    p_telemetry->actuator_left_duty_permille = (int16_t) acoustic_read_u16_le(&p_frame->payload[16]);
    p_telemetry->actuator_right_duty_permille = (int16_t) acoustic_read_u16_le(&p_frame->payload[18]);
    p_telemetry->actuator_left_encoder_rpm_x10 = (int16_t) acoustic_read_u16_le(&p_frame->payload[20]);
    p_telemetry->actuator_right_encoder_rpm_x10 = (int16_t) acoustic_read_u16_le(&p_frame->payload[22]);
    return true;
}

/** =================================================================*
 * @brief  オドメトリ位置姿勢テレメトリ復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_telemetry オドメトリ位置姿勢テレメトリ
 * @return フレームが正しい位置姿勢テレメトリならtrue
 * ================================================================= */
bool acoustic_protocol_decode_pose_telemetry(const acoustic_frame_t * p_frame,
                                             acoustic_pose_telemetry_t * p_telemetry) {
    if ((NULL == p_frame) || (NULL == p_telemetry) || (ACOUSTIC_MESSAGE_POSE_TELEMETRY != p_frame->type) ||
        (ACOUSTIC_POSE_TELEMETRY_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_telemetry->x_mm = (int32_t) acoustic_read_u32_le(&p_frame->payload[0]);
    p_telemetry->y_mm = (int32_t) acoustic_read_u32_le(&p_frame->payload[4]);
    p_telemetry->theta_mrad = (int32_t) acoustic_read_u32_le(&p_frame->payload[8]);
    p_telemetry->v_mm_s = (int32_t) acoustic_read_u32_le(&p_frame->payload[12]);
    p_telemetry->omega_mrad_s = (int32_t) acoustic_read_u32_le(&p_frame->payload[16]);
    p_telemetry->left_encoder_count = (int32_t) acoustic_read_u32_le(&p_frame->payload[20]);
    p_telemetry->right_encoder_count = (int32_t) acoustic_read_u32_le(&p_frame->payload[24]);
    p_telemetry->gyro_bias_dps_x10 = (int16_t) acoustic_read_u16_le(&p_frame->payload[28]);
    p_telemetry->flags = p_frame->payload[30];
    p_telemetry->reserved = p_frame->payload[31];
    p_telemetry->uptime_ms = acoustic_read_u32_le(&p_frame->payload[32]);
    return true;
}

/** =================================================================*
 * @brief  音源位置・到着・回避診断情報を復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_diagnostics 音源ナビゲーション診断
 * @return フレームが正しい診断情報ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_nav_diagnostics(const acoustic_frame_t * p_frame,
                                             acoustic_nav_diagnostics_t * p_diagnostics) {
    if ((NULL == p_frame) || (NULL == p_diagnostics) || (ACOUSTIC_MESSAGE_NAV_DIAGNOSTICS != p_frame->type) ||
        (ACOUSTIC_NAV_DIAGNOSTICS_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    p_diagnostics->schema_version = p_frame->payload[0];
    p_diagnostics->flags = p_frame->payload[1];
    p_diagnostics->doa_confidence = p_frame->payload[2];
    p_diagnostics->arrival_state = p_frame->payload[3];
    p_diagnostics->observation_sequence = acoustic_read_u32_le(&p_frame->payload[4]);
    p_diagnostics->raw_doa_deg = acoustic_read_u16_le(&p_frame->payload[8]);
    p_diagnostics->filtered_doa_deg = acoustic_read_u16_le(&p_frame->payload[10]);
    p_diagnostics->rover_x_mm = (int32_t) acoustic_read_u32_le(&p_frame->payload[12]);
    p_diagnostics->rover_y_mm = (int32_t) acoustic_read_u32_le(&p_frame->payload[16]);
    p_diagnostics->rover_heading_mrad = (int32_t) acoustic_read_u32_le(&p_frame->payload[20]);
    p_diagnostics->source_x_mm = (int32_t) acoustic_read_u32_le(&p_frame->payload[24]);
    p_diagnostics->source_y_mm = (int32_t) acoustic_read_u32_le(&p_frame->payload[28]);
    p_diagnostics->source_range_mm = acoustic_read_u32_le(&p_frame->payload[32]);
    p_diagnostics->source_bearing_deg = (int16_t) acoustic_read_u16_le(&p_frame->payload[36]);
    p_diagnostics->source_confidence = p_frame->payload[38];
    p_diagnostics->observation_count = p_frame->payload[39];
    p_diagnostics->localization_residual_mm = acoustic_read_u16_le(&p_frame->payload[40]);
    p_diagnostics->crossing_angle_deg = acoustic_read_u16_le(&p_frame->payload[42]);
    p_diagnostics->baseline_mm = acoustic_read_u16_le(&p_frame->payload[44]);
    p_diagnostics->source_position_shift_mm = acoustic_read_u16_le(&p_frame->payload[46]);
    p_diagnostics->arrival_confirm_count = p_frame->payload[48];
    p_diagnostics->sensor_rule = p_frame->payload[49];
    p_diagnostics->think_state = p_frame->payload[50];
    p_diagnostics->reserved = p_frame->payload[51];
    p_diagnostics->autonomous_backup_count = acoustic_read_u32_le(&p_frame->payload[52]);
    return true;
}

/** =================================================================*
 * @brief AI Lab推論snapshotを復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_snapshot snapshot情報
 * @return snapshot形式が正しければtrue
 * ================================================================= */
bool acoustic_protocol_decode_ai_lab_snapshot(const acoustic_frame_t * p_frame,
                                              acoustic_ai_lab_snapshot_t * p_snapshot) {
    if ((NULL == p_frame) || (NULL == p_snapshot) ||
        (ACOUSTIC_MESSAGE_AI_LAB_SNAPSHOT != p_frame->type) ||
        (ACOUSTIC_AI_LAB_SNAPSHOT_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }

    const uint8_t * payload = p_frame->payload;
    memset(p_snapshot, 0, sizeof(*p_snapshot));
    p_snapshot->schema_version = payload[0];
    p_snapshot->flags = payload[1];
    p_snapshot->think_state = payload[2];
    p_snapshot->infer_status = payload[3];
    p_snapshot->doa_deg = acoustic_read_u16_le(&payload[4]);
    p_snapshot->level_dbfs_x100 = (int16_t) acoustic_read_u16_le(&payload[6]);
    p_snapshot->peak_dbfs_x100 = (int16_t) acoustic_read_u16_le(&payload[8]);
    p_snapshot->vad = payload[10];
    p_snapshot->xvf_status = payload[11];
    p_snapshot->audio_flags = payload[12];
    p_snapshot->learning_samples = payload[13];
    p_snapshot->target_peak_bin = payload[14];
    p_snapshot->current_peak_bin = payload[15];
    p_snapshot->nearest_sample = payload[16];
    p_snapshot->active_frame_count = payload[17];
    p_snapshot->cosine_distance_x1000 = acoustic_read_u16_le(&payload[18]);
    p_snapshot->identifier_threshold_x1000 = acoustic_read_u16_le(&payload[20]);
    p_snapshot->similarity_permille = acoustic_read_u16_le(&payload[22]);
    p_snapshot->background_mse_x1000 = acoustic_read_u16_le(&payload[24]);
    p_snapshot->background_threshold_x1000 = acoustic_read_u16_le(&payload[26]);
    p_snapshot->observation_sequence = acoustic_read_u32_le(&payload[28]);
    p_snapshot->feature_generation = acoustic_read_u32_le(&payload[32]);
    p_snapshot->inference_count = acoustic_read_u32_le(&payload[36]);
    p_snapshot->match_count = acoustic_read_u32_le(&payload[40]);
    p_snapshot->storage_result = payload[44];
    p_snapshot->reserved[0] = payload[45];
    p_snapshot->reserved[1] = payload[46];
    p_snapshot->reserved[2] = payload[47];
    return true;
}

/** =================================================================*
 * @brief AI Lab要約chunkを復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_chunk 受信chunk
 * @return chunk形式が正しければtrue
 * ================================================================= */
bool acoustic_protocol_decode_ai_lab_summary_chunk(const acoustic_frame_t * p_frame,
                                                   acoustic_ai_lab_summary_chunk_t * p_chunk) {
    if ((NULL == p_frame) || (NULL == p_chunk) ||
        (ACOUSTIC_MESSAGE_AI_LAB_SUMMARY_CHUNK != p_frame->type) ||
        (ACOUSTIC_AI_LAB_SUMMARY_CHUNK_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }
    p_chunk->feature_generation = acoustic_read_u32_le(&p_frame->payload[0]);
    p_chunk->chunk_index = p_frame->payload[4];
    p_chunk->chunk_count = p_frame->payload[5];
    p_chunk->schema_version = p_frame->payload[6];
    p_chunk->cpu_drop_count = p_frame->payload[7];
    memcpy(p_chunk->data, &p_frame->payload[8], sizeof(p_chunk->data));
    return true;
}

/** =================================================================*
 * @brief AI Lab保存見本chunkを復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_chunk 受信chunk
 * @return chunk形式が正しければtrue
 * ================================================================= */
bool acoustic_protocol_decode_ai_lab_profile_chunk(const acoustic_frame_t * p_frame,
                                                   acoustic_ai_lab_profile_chunk_t * p_chunk) {
    if ((NULL == p_frame) || (NULL == p_chunk) ||
        (ACOUSTIC_MESSAGE_AI_LAB_PROFILE_CHUNK != p_frame->type) ||
        (ACOUSTIC_AI_LAB_PROFILE_CHUNK_PAYLOAD_SIZE != p_frame->payload_length)) {
        return false;
    }
    p_chunk->profile_generation = acoustic_read_u32_le(&p_frame->payload[0]);
    p_chunk->sample_index = p_frame->payload[4];
    p_chunk->chunk_index = p_frame->payload[5];
    p_chunk->chunk_count = p_frame->payload[6];
    p_chunk->sample_count = p_frame->payload[7];
    memcpy(p_chunk->data, &p_frame->payload[8], sizeof(p_chunk->data));
    return true;
}
