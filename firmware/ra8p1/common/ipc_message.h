/** =================================================================*
 * @file   ipc_message.h
 * @brief  CPU0・CPU1間IPCメッセージ定義
 * ================================================================= */
#ifndef SEROV_IPC_MESSAGE_H
#define SEROV_IPC_MESSAGE_H

#include <tk/tkernel.h>                                     /* μT-Kernelの基本型、インライン定義 */

#define ACTUATOR_SERVO_COUNT                  (4U)

/* servo_target_degの配列順: FR、FL、RR、RL。 */

/* CPU0からCPU1へ送る意味ベースのアクチュエータ指令。 */
typedef struct st_actuator_command {
    H  left_target_rpm;
    H  right_target_rpm;
    H  servo_target_deg[ACTUATOR_SERVO_COUNT];
    UB actuator_enable;
    UB emergency_stop;
    UW sequence_number;
} actuator_command_t;

/* CPU1で実際に適用しているアクチュエータ状態。 */
typedef struct st_actuator_status {
    H  left_duty_permille;
    H  right_duty_permille;
    H  left_encoder_rpm_x10;
    H  right_encoder_rpm_x10;
    UH fault_flags;
    UW applied_command_sequence;
    UW sequence_number;
} actuator_status_t;

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
#define ACTUATOR_IPC_STATUS_WORD_COUNT      (7U)

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
    ACTUATOR_IPC_STATUS_FAULT_FLAGS         = 0x81U,
    ACTUATOR_IPC_STATUS_LEFT_DUTY           = 0x82U,
    ACTUATOR_IPC_STATUS_RIGHT_DUTY          = 0x83U,
    ACTUATOR_IPC_STATUS_LEFT_ENCODER_RPM_X10  = 0x84U,
    ACTUATOR_IPC_STATUS_RIGHT_ENCODER_RPM_X10 = 0x85U,
    ACTUATOR_IPC_STATUS_APPLIED_SEQUENCE    = 0x86U,
    ACTUATOR_IPC_STATUS_SEQUENCE            = 0x87U,
} actuator_ipc_message_id_t;

/** =================================================================*
 * @brief  安全アクチュエータ指令生成
 * @return 緊急停止状態の指令
 * ================================================================= */
Inline actuator_command_t actuator_command_make_safe(void) {
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
Inline UW actuator_ipc_make_word(actuator_ipc_message_id_t id, UW payload) {
    return ((UW) id << ACTUATOR_IPC_MESSAGE_ID_SHIFT) |
           (payload & ACTUATOR_IPC_PAYLOAD_MASK);
}

/** =================================================================*
 * @brief  制御指令通信語生成
 * @param[in] enable アクチュエータ有効状態
 * @param[in] emergency_stop 緊急停止状態
 * @return 制御指令のIPC通信語
 * ================================================================= */
Inline UW actuator_ipc_make_control_word(BOOL enable, BOOL emergency_stop) {
    UW payload = enable ? ACTUATOR_CONTROL_ENABLE_MASK : 0U;
    payload |= emergency_stop ? ACTUATOR_CONTROL_EMERGENCY_STOP_MASK : 0U;
    return actuator_ipc_make_word(ACTUATOR_IPC_COMMAND_CONTROL, payload);
}

/** =================================================================*
 * @brief  16 bit値通信語生成
 * @param[in] id メッセージID
 * @param[in] value 符号付き16 bit値
 * @return 16 bit値を含むIPC通信語
 * ================================================================= */
Inline UW actuator_ipc_make_i16_word(actuator_ipc_message_id_t id, H value) {
    return actuator_ipc_make_word(id, (UW) (UH) value);
}

/** =================================================================*
 * @brief  シーケンス通信語生成
 * @param[in] sequence_number シーケンス番号
 * @return シーケンス番号を含むIPC通信語
 * ================================================================= */
Inline UW actuator_ipc_make_sequence_word(UW sequence_number) {
    return actuator_ipc_make_word(ACTUATOR_IPC_COMMAND_SEQUENCE,
                                  sequence_number & ACTUATOR_IPC_SEQUENCE_MASK);
}

/** =================================================================*
 * @brief  CPU1状態シーケンス通信語生成
 * @param[in] sequence_number 状態シーケンス番号
 * @return シーケンス番号を含むIPC通信語
 * ================================================================= */
Inline UW actuator_ipc_make_status_sequence_word(UW sequence_number) {
    return actuator_ipc_make_word(ACTUATOR_IPC_STATUS_SEQUENCE, sequence_number & ACTUATOR_IPC_SEQUENCE_MASK);
}

/** =================================================================*
 * @brief  IPC通信語からメッセージID取得
 * @param[in] word IPC通信語
 * @return メッセージID
 * ================================================================= */
Inline actuator_ipc_message_id_t actuator_ipc_get_message_id(UW word) {
    return (actuator_ipc_message_id_t) ((word & ACTUATOR_IPC_MESSAGE_ID_MASK) >>
                                        ACTUATOR_IPC_MESSAGE_ID_SHIFT);
}

/** =================================================================*
 * @brief  IPC通信語からペイロード取得
 * @param[in] word IPC通信語
 * @return 24 bitペイロード
 * ================================================================= */
Inline UW actuator_ipc_get_payload(UW word) {
    return word & ACTUATOR_IPC_PAYLOAD_MASK;
}

/** =================================================================*
 * @brief  IPC通信語から16 bit値取得
 * @param[in] word IPC通信語
 * @return 符号付き16 bit値
 * ================================================================= */
Inline H actuator_ipc_get_i16_payload(UW word) {
    return (H) (UH) actuator_ipc_get_payload(word);
}

#endif /* SEROV_IPC_MESSAGE_H */
