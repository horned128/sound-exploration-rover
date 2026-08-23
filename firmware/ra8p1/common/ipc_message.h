/** =================================================================*
 * @file   ipc_message.h
 * @brief  CPU0・CPU1間IPCメッセージ定義
 * ================================================================= */
#ifndef SEROV_IPC_MESSAGE_H
#define SEROV_IPC_MESSAGE_H

#include <stdbool.h>                                        /* bool型 */
#include <stdint.h>                                         /* 固定幅整数型 */

#define ACTUATOR_SERVO_COUNT                  (4U)

/* servo_target_degの配列順: FR、FL、RR、RL。 */

/* CPU0からCPU1へ送る意味ベースのアクチュエータ指令。 */
typedef struct st_actuator_command {
    int16_t  left_target_rpm;
    int16_t  right_target_rpm;
    int16_t  servo_target_deg[ACTUATOR_SERVO_COUNT];
    uint8_t  actuator_enable;
    uint8_t  emergency_stop;
    uint32_t sequence_number;
} actuator_command_t;

typedef enum e_actuator_fault {
    ACTUATOR_FAULT_NONE                    = 0U,
    ACTUATOR_FAULT_COMMAND_TIMEOUT         = (1U << 0),
    ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE   = (1U << 1),
    ACTUATOR_FAULT_COMMAND_LIMITED         = (1U << 2),
    ACTUATOR_FAULT_IPC_RX                  = (1U << 3),
    ACTUATOR_FAULT_DRIVER                  = (1U << 4),
} actuator_fault_t;

/* 通信語形式: [31:24] メッセージID、[23:0] ペイロード */
#define ACTUATOR_IPC_MESSAGE_ID_SHIFT       (24U)
#define ACTUATOR_IPC_MESSAGE_ID_MASK        (0xFF000000UL)
#define ACTUATOR_IPC_PAYLOAD_MASK           (0x00FFFFFFUL)
#define ACTUATOR_IPC_SEQUENCE_MASK          (0x00FFFFFFUL)

#define ACTUATOR_CONTROL_ENABLE_MASK         (1UL << 0)
#define ACTUATOR_CONTROL_EMERGENCY_STOP_MASK (1UL << 1)

typedef enum e_actuator_ipc_message_id {
    ACTUATOR_IPC_COMMAND_CONTROL             = 0x01U,
    ACTUATOR_IPC_COMMAND_LEFT_TARGET_RPM     = 0x02U,
    ACTUATOR_IPC_COMMAND_RIGHT_TARGET_RPM    = 0x03U,
    ACTUATOR_IPC_COMMAND_FR_TARGET_DEG      = 0x04U,
    ACTUATOR_IPC_COMMAND_SEQUENCE            = 0x05U,
    ACTUATOR_IPC_COMMAND_FL_TARGET_DEG      = 0x06U,
    ACTUATOR_IPC_COMMAND_RR_TARGET_DEG      = 0x07U,
    ACTUATOR_IPC_COMMAND_RL_TARGET_DEG      = 0x08U,

} actuator_ipc_message_id_t;

/** =================================================================*
 * @brief  安全アクチュエータ指令生成
 * @return 緊急停止状態の指令
 * ================================================================= */
static inline actuator_command_t actuator_command_make_safe(void) {
    actuator_command_t command = {0};
    command.emergency_stop = 1U;
    return command;
}

/** =================================================================*
 * @brief  IPC通信語生成
 * @param[in] id メッセージID
 * @param[in] payload 24 bitペイロード
 * @return エンコード済みIPC通信語
 * ================================================================= */
static inline uint32_t actuator_ipc_make_word(actuator_ipc_message_id_t id, uint32_t payload) {
    return ((uint32_t) id << ACTUATOR_IPC_MESSAGE_ID_SHIFT) |
           (payload & ACTUATOR_IPC_PAYLOAD_MASK);
}

/** =================================================================*
 * @brief  制御指令通信語生成
 * @param[in] enable アクチュエータ有効状態
 * @param[in] emergency_stop 緊急停止状態
 * @return 制御指令のIPC通信語
 * ================================================================= */
static inline uint32_t actuator_ipc_make_control_word(bool enable, bool emergency_stop) {
    uint32_t payload = enable ? ACTUATOR_CONTROL_ENABLE_MASK : 0U;
    payload |= emergency_stop ? ACTUATOR_CONTROL_EMERGENCY_STOP_MASK : 0U;
    return actuator_ipc_make_word(ACTUATOR_IPC_COMMAND_CONTROL, payload);
}

/** =================================================================*
 * @brief  16 bit値通信語生成
 * @param[in] id メッセージID
 * @param[in] value 符号付き16 bit値
 * @return 16 bit値を含むIPC通信語
 * ================================================================= */
static inline uint32_t actuator_ipc_make_i16_word(actuator_ipc_message_id_t id, int16_t value) {
    return actuator_ipc_make_word(id, (uint32_t) (uint16_t) value);
}

/** =================================================================*
 * @brief  シーケンス通信語生成
 * @param[in] sequence_number シーケンス番号
 * @return シーケンス番号を含むIPC通信語
 * ================================================================= */
static inline uint32_t actuator_ipc_make_sequence_word(uint32_t sequence_number) {
    return actuator_ipc_make_word(ACTUATOR_IPC_COMMAND_SEQUENCE,
                                  sequence_number & ACTUATOR_IPC_SEQUENCE_MASK);
}

/** =================================================================*
 * @brief  IPC通信語からメッセージID取得
 * @param[in] word IPC通信語
 * @return メッセージID
 * ================================================================= */
static inline actuator_ipc_message_id_t actuator_ipc_get_message_id(uint32_t word) {
    return (actuator_ipc_message_id_t) ((word & ACTUATOR_IPC_MESSAGE_ID_MASK) >>
                                        ACTUATOR_IPC_MESSAGE_ID_SHIFT);
}

/** =================================================================*
 * @brief  IPC通信語からペイロード取得
 * @param[in] word IPC通信語
 * @return 24 bitペイロード
 * ================================================================= */
static inline uint32_t actuator_ipc_get_payload(uint32_t word) {
    return word & ACTUATOR_IPC_PAYLOAD_MASK;
}

/** =================================================================*
 * @brief  IPC通信語から16 bit値取得
 * @param[in] word IPC通信語
 * @return 符号付き16 bit値
 * ================================================================= */
static inline int16_t actuator_ipc_get_i16_payload(uint32_t word) {
    return (int16_t) (uint16_t) actuator_ipc_get_payload(word);
}

#endif /* SEROV_IPC_MESSAGE_H */
