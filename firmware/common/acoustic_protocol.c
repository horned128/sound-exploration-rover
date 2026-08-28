/** =================================================================*
 * @file   acoustic_protocol.c
 * @brief  ReSpeaker・RA8P1間音響通信プロトコル処理
 * ================================================================= */
#include "acoustic_protocol.h"                             /* 音響通信の型、定数、API */
#include <string.h>                                        /* memcpy、memset */

#define ACOUSTIC_FRAME_VERSION_OFFSET             (2U)
#define ACOUSTIC_FRAME_TYPE_OFFSET                (3U)
#define ACOUSTIC_FRAME_LENGTH_OFFSET              (4U)
#define ACOUSTIC_FRAME_SEQUENCE_OFFSET            (6U)
#define ACOUSTIC_FRAME_UPTIME_OFFSET              (10U)
#define ACOUSTIC_FRAME_PAYLOAD_OFFSET             \
    ACOUSTIC_PROTOCOL_HEADER_SIZE

static void acoustic_write_u16_le(uint8_t * p_output,
                                  uint16_t value);          /* 16 bit LE書込 */
static void acoustic_write_u32_le(uint8_t * p_output,
                                  uint32_t value);          /* 32 bit LE書込 */
static uint16_t acoustic_read_u16_le(
    const uint8_t * p_input);                              /* 16 bit LE読出 */
static uint32_t acoustic_read_u32_le(
    const uint8_t * p_input);                              /* 32 bit LE読出 */
static void acoustic_protocol_parser_resync(
    acoustic_protocol_parser_t * p_parser,
    uint8_t byte);                                         /* magic再同期 */

/** =================================================================*
 * @brief  16 bit little-endian書込
 * @param[out] p_output 2 byte以上の出力先
 * @param[in] value 書き込む値
 * ================================================================= */
static void acoustic_write_u16_le(uint8_t * p_output,
                                  uint16_t value) {
    p_output[0] = (uint8_t) value;
    p_output[1] = (uint8_t) (value >> 8U);
}

/** =================================================================*
 * @brief  32 bit little-endian書込
 * @param[out] p_output 4 byte以上の出力先
 * @param[in] value 書き込む値
 * ================================================================= */
static void acoustic_write_u32_le(uint8_t * p_output,
                                  uint32_t value) {
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
    return (uint16_t) ((uint16_t) p_input[0] |
                       ((uint16_t) p_input[1] << 8U));
}

/** =================================================================*
 * @brief  32 bit little-endian読出
 * @param[in] p_input 4 byte以上の入力元
 * @return 読み出した値
 * ================================================================= */
static uint32_t acoustic_read_u32_le(const uint8_t * p_input) {
    return (uint32_t) p_input[0] |
           ((uint32_t) p_input[1] << 8U) |
           ((uint32_t) p_input[2] << 16U) |
           ((uint32_t) p_input[3] << 24U);
}

/** =================================================================*
 * @brief  CRC-16/CCITT-FALSE計算
 * @param[in] p_data 入力データ
 * @param[in] length 入力長
 * @return CRC値
 * ================================================================= */
uint16_t acoustic_protocol_crc16(const uint8_t * p_data,
                                 size_t length) {
    uint16_t crc = 0xFFFFU;

    if ((NULL == p_data) && (0U != length)) {
        return 0U;
    }

    for (size_t index = 0U; index < length; index++) {
        crc ^= (uint16_t) p_data[index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (0U != (crc & 0x8000U)) ?
                  (uint16_t) ((crc << 1U) ^ 0x1021U) :
                  (uint16_t) (crc << 1U);
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
size_t acoustic_protocol_encode(acoustic_message_type_t type,
                                uint32_t sequence,
                                uint32_t uptime_ms,
                                const uint8_t * p_payload,
                                uint16_t payload_length,
                                uint8_t * p_output,
                                size_t output_capacity) {
    size_t const frame_size = ACOUSTIC_PROTOCOL_HEADER_SIZE +
                              (size_t) payload_length +
                              ACOUSTIC_PROTOCOL_CRC_SIZE;

    if ((NULL == p_output) ||
        ((NULL == p_payload) && (0U != payload_length)) ||
        (0U == (uint8_t) type) ||
        (payload_length > ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE) ||
        (output_capacity < frame_size)) {
        return 0U;
    }

    p_output[0] = ACOUSTIC_PROTOCOL_MAGIC_0;
    p_output[1] = ACOUSTIC_PROTOCOL_MAGIC_1;
    p_output[ACOUSTIC_FRAME_VERSION_OFFSET] =
        ACOUSTIC_PROTOCOL_VERSION;
    p_output[ACOUSTIC_FRAME_TYPE_OFFSET] = (uint8_t) type;
    acoustic_write_u16_le(&p_output[ACOUSTIC_FRAME_LENGTH_OFFSET],
                          payload_length);
    acoustic_write_u32_le(&p_output[ACOUSTIC_FRAME_SEQUENCE_OFFSET],
                          sequence);
    acoustic_write_u32_le(&p_output[ACOUSTIC_FRAME_UPTIME_OFFSET],
                          uptime_ms);
    if (0U != payload_length) {
        memcpy(&p_output[ACOUSTIC_FRAME_PAYLOAD_OFFSET],
               p_payload,
               payload_length);
    }

    uint16_t const crc = acoustic_protocol_crc16(
        &p_output[ACOUSTIC_FRAME_VERSION_OFFSET],
        frame_size - ACOUSTIC_FRAME_VERSION_OFFSET -
        ACOUSTIC_PROTOCOL_CRC_SIZE);
    acoustic_write_u16_le(&p_output[frame_size -
                                    ACOUSTIC_PROTOCOL_CRC_SIZE],
                          crc);
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
size_t acoustic_protocol_encode_observation(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_observation_t * p_observation,
    uint8_t * p_output,
    size_t output_capacity) {
    if (NULL == p_observation) {
        return 0U;
    }

    uint8_t payload[ACOUSTIC_OBSERVATION_PAYLOAD_SIZE];
    acoustic_write_u16_le(&payload[0], p_observation->doa_deg);
    acoustic_write_u16_le(&payload[2],
                          (uint16_t) p_observation->level_dbfs_x100);
    acoustic_write_u16_le(&payload[4],
                          (uint16_t) p_observation->peak_dbfs_x100);
    payload[6] = p_observation->vad;
    payload[7] = p_observation->xvf_status;
    payload[8] = p_observation->audio_flags;
    payload[9] = p_observation->xvf_raw_status;
    acoustic_write_u32_le(&payload[10],
                          p_observation->audio_frame_count);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_OBSERVATION,
                                    sequence,
                                    uptime_ms,
                                    payload,
                                    (uint16_t) sizeof(payload),
                                    p_output,
                                    output_capacity);
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
size_t acoustic_protocol_encode_hello(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_hello_t * p_hello,
    uint8_t * p_output,
    size_t output_capacity) {
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

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_HELLO,
                                    sequence,
                                    uptime_ms,
                                    payload,
                                    (uint16_t) sizeof(payload),
                                    p_output,
                                    output_capacity);
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
size_t acoustic_protocol_encode_health(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_health_t * p_health,
    uint8_t * p_output,
    size_t output_capacity) {
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

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_HEALTH,
                                    sequence,
                                    uptime_ms,
                                    payload,
                                    (uint16_t) sizeof(payload),
                                    p_output,
                                    output_capacity);
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
size_t acoustic_protocol_encode_rover_telemetry(
    uint32_t sequence,
    uint32_t uptime_ms,
    const acoustic_rover_telemetry_t * p_telemetry,
    uint8_t * p_output,
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
    acoustic_write_u16_le(&payload[6],
                          (uint16_t) p_telemetry->level_dbfs_x100);
    acoustic_write_u16_le(&payload[8],
                          (uint16_t) p_telemetry->peak_dbfs_x100);
    payload[10] = p_telemetry->vad;
    payload[11] = p_telemetry->xvf_status;
    payload[12] = p_telemetry->audio_flags;
    payload[13] = p_telemetry->xvf_raw_status;
    acoustic_write_u32_le(&payload[14],
                          p_telemetry->observation_sequence);
    acoustic_write_u32_le(&payload[18],
                          p_telemetry->observation_age_ms);
    acoustic_write_u32_le(&payload[22],
                          p_telemetry->audio_frame_count);
    acoustic_write_u32_le(&payload[26],
                          p_telemetry->audio_crc_error_count);
    acoustic_write_u32_le(&payload[30], p_telemetry->fault_flags);
    acoustic_write_u16_le(&payload[34],
                          (uint16_t) p_telemetry->steering_deg);
    acoustic_write_u16_le(&payload[36],
                          (uint16_t) p_telemetry->left_target_rpm);
    acoustic_write_u16_le(&payload[38],
                          (uint16_t) p_telemetry->right_target_rpm);
    for (uint32_t index = 0U; index < 4U; index++) {
        acoustic_write_u16_le(&payload[40U + (index * 2U)],
            (uint16_t) p_telemetry->servo_target_deg[index]);
    }
    acoustic_write_u32_le(&payload[48], p_telemetry->command_sequence);
    acoustic_write_u32_le(&payload[52],
                          (uint32_t) p_telemetry->command_last_error);
    acoustic_write_u32_le(&payload[56],
                          p_telemetry->command_target_age_ms);
    acoustic_write_u32_le(&payload[60],
                          p_telemetry->command_send_count);

    return acoustic_protocol_encode(ACOUSTIC_MESSAGE_ROVER_TELEMETRY,
                                    sequence,
                                    uptime_ms,
                                    payload,
                                    (uint16_t) sizeof(payload),
                                    p_output,
                                    output_capacity);
}

/** =================================================================*
 * @brief  パーサー初期化
 * @param[out] p_parser パーサー状態
 * ================================================================= */
void acoustic_protocol_parser_init(
    acoustic_protocol_parser_t * p_parser) {
    if (NULL != p_parser) {
        memset(p_parser, 0, sizeof(*p_parser));
    }
}

/** =================================================================*
 * @brief  magic再同期
 * @param[in,out] p_parser パーサー状態
 * @param[in] byte 現在の入力byte
 * ================================================================= */
static void acoustic_protocol_parser_resync(
    acoustic_protocol_parser_t * p_parser,
    uint8_t byte) {
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
acoustic_parse_result_t acoustic_protocol_parser_push(
    acoustic_protocol_parser_t * p_parser,
    uint8_t byte,
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
        uint16_t const payload_length = acoustic_read_u16_le(
            &p_parser->bytes[ACOUSTIC_FRAME_LENGTH_OFFSET]);
        if (payload_length > ACOUSTIC_PROTOCOL_MAX_PAYLOAD_SIZE) {
            acoustic_protocol_parser_resync(p_parser, byte);
            return ACOUSTIC_PARSE_FORMAT_ERROR;
        }
        if (ACOUSTIC_PROTOCOL_VERSION !=
            p_parser->bytes[ACOUSTIC_FRAME_VERSION_OFFSET]) {
            acoustic_protocol_parser_resync(p_parser, byte);
            return ACOUSTIC_PARSE_UNSUPPORTED_VERSION;
        }
        p_parser->expected = (uint16_t) (
            ACOUSTIC_PROTOCOL_HEADER_SIZE + payload_length +
            ACOUSTIC_PROTOCOL_CRC_SIZE);
    }

    if ((0U == p_parser->expected) ||
        (p_parser->used < p_parser->expected)) {
        return ACOUSTIC_PARSE_MORE;
    }

    uint16_t const received_crc = acoustic_read_u16_le(
        &p_parser->bytes[p_parser->expected -
                         ACOUSTIC_PROTOCOL_CRC_SIZE]);
    uint16_t const calculated_crc = acoustic_protocol_crc16(
        &p_parser->bytes[ACOUSTIC_FRAME_VERSION_OFFSET],
        p_parser->expected - ACOUSTIC_FRAME_VERSION_OFFSET -
        ACOUSTIC_PROTOCOL_CRC_SIZE);
    if (received_crc != calculated_crc) {
        acoustic_protocol_parser_resync(p_parser, byte);
        return ACOUSTIC_PARSE_CRC_ERROR;
    }

    p_frame->version =
        p_parser->bytes[ACOUSTIC_FRAME_VERSION_OFFSET];
    p_frame->type = (acoustic_message_type_t)
        p_parser->bytes[ACOUSTIC_FRAME_TYPE_OFFSET];
    p_frame->payload_length = acoustic_read_u16_le(
        &p_parser->bytes[ACOUSTIC_FRAME_LENGTH_OFFSET]);
    p_frame->sequence = acoustic_read_u32_le(
        &p_parser->bytes[ACOUSTIC_FRAME_SEQUENCE_OFFSET]);
    p_frame->uptime_ms = acoustic_read_u32_le(
        &p_parser->bytes[ACOUSTIC_FRAME_UPTIME_OFFSET]);
    if (0U != p_frame->payload_length) {
        memcpy(p_frame->payload,
               &p_parser->bytes[ACOUSTIC_FRAME_PAYLOAD_OFFSET],
               p_frame->payload_length);
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
bool acoustic_protocol_decode_observation(
    const acoustic_frame_t * p_frame,
    acoustic_observation_t * p_observation) {
    if ((NULL == p_frame) || (NULL == p_observation) ||
        (ACOUSTIC_MESSAGE_OBSERVATION != p_frame->type) ||
        (ACOUSTIC_OBSERVATION_PAYLOAD_SIZE !=
         p_frame->payload_length)) {
        return false;
    }

    p_observation->doa_deg = acoustic_read_u16_le(&p_frame->payload[0]);
    p_observation->level_dbfs_x100 =
        (int16_t) acoustic_read_u16_le(&p_frame->payload[2]);
    p_observation->peak_dbfs_x100 =
        (int16_t) acoustic_read_u16_le(&p_frame->payload[4]);
    p_observation->vad = p_frame->payload[6];
    p_observation->xvf_status = p_frame->payload[7];
    p_observation->audio_flags = p_frame->payload[8];
    p_observation->xvf_raw_status = p_frame->payload[9];
    p_observation->audio_frame_count =
        acoustic_read_u32_le(&p_frame->payload[10]);
    return true;
}

/** =================================================================*
 * @brief  起動情報復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_hello 起動情報
 * @return フレームが正しい起動情報ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_hello(const acoustic_frame_t * p_frame,
                                    acoustic_hello_t * p_hello) {
    if ((NULL == p_frame) || (NULL == p_hello) ||
        (ACOUSTIC_MESSAGE_HELLO != p_frame->type) ||
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
bool acoustic_protocol_decode_health(const acoustic_frame_t * p_frame,
                                     acoustic_health_t * p_health) {
    if ((NULL == p_frame) || (NULL == p_health) ||
        (ACOUSTIC_MESSAGE_HEALTH != p_frame->type) ||
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
 * @brief  ローバ診断情報復号
 * @param[in] p_frame 受信フレーム
 * @param[out] p_telemetry ローバ診断情報
 * @return フレームが正しいローバ診断情報ならtrue
 * ================================================================= */
bool acoustic_protocol_decode_rover_telemetry(
    const acoustic_frame_t * p_frame,
    acoustic_rover_telemetry_t * p_telemetry) {
    if ((NULL == p_frame) || (NULL == p_telemetry) ||
        (ACOUSTIC_MESSAGE_ROVER_TELEMETRY != p_frame->type) ||
        (ACOUSTIC_ROVER_TELEMETRY_PAYLOAD_SIZE !=
         p_frame->payload_length)) {
        return false;
    }

    p_telemetry->schema_version = p_frame->payload[0];
    p_telemetry->think_state = p_frame->payload[1];
    p_telemetry->usb_state = p_frame->payload[2];
    p_telemetry->flags = p_frame->payload[3];
    p_telemetry->doa_deg = acoustic_read_u16_le(&p_frame->payload[4]);
    p_telemetry->level_dbfs_x100 =
        (int16_t) acoustic_read_u16_le(&p_frame->payload[6]);
    p_telemetry->peak_dbfs_x100 =
        (int16_t) acoustic_read_u16_le(&p_frame->payload[8]);
    p_telemetry->vad = p_frame->payload[10];
    p_telemetry->xvf_status = p_frame->payload[11];
    p_telemetry->audio_flags = p_frame->payload[12];
    p_telemetry->xvf_raw_status = p_frame->payload[13];
    p_telemetry->observation_sequence =
        acoustic_read_u32_le(&p_frame->payload[14]);
    p_telemetry->observation_age_ms =
        acoustic_read_u32_le(&p_frame->payload[18]);
    p_telemetry->audio_frame_count =
        acoustic_read_u32_le(&p_frame->payload[22]);
    p_telemetry->audio_crc_error_count =
        acoustic_read_u32_le(&p_frame->payload[26]);
    p_telemetry->fault_flags =
        acoustic_read_u32_le(&p_frame->payload[30]);
    p_telemetry->steering_deg =
        (int16_t) acoustic_read_u16_le(&p_frame->payload[34]);
    p_telemetry->left_target_rpm =
        (int16_t) acoustic_read_u16_le(&p_frame->payload[36]);
    p_telemetry->right_target_rpm =
        (int16_t) acoustic_read_u16_le(&p_frame->payload[38]);
    for (uint32_t index = 0U; index < 4U; index++) {
        p_telemetry->servo_target_deg[index] =
            (int16_t) acoustic_read_u16_le(
                &p_frame->payload[40U + (index * 2U)]);
    }
    p_telemetry->command_sequence =
        acoustic_read_u32_le(&p_frame->payload[48]);
    p_telemetry->command_last_error =
        (int32_t) acoustic_read_u32_le(&p_frame->payload[52]);
    p_telemetry->command_target_age_ms =
        acoustic_read_u32_le(&p_frame->payload[56]);
    p_telemetry->command_send_count =
        acoustic_read_u32_le(&p_frame->payload[60]);
    return true;
}
