/** =================================================================*
 * @file   actuator_service.c
 * @brief  CPU1アクチュエータアプリケーション
 * ================================================================= */
#include "actuator_service.h"                              /* CPU1アクチュエータアプリケーションAPI */
#include "config/actuator_config.h"                        /* アクチュエータ安全設定 */
#include "config/drive_config.h"                           /* 駆動目標の上限値 */
#include "services/drive_service.h"                        /* 左右drive service API */
#include "drivers/encoder.h"                               /* エンコーダ取得API */
#include "drivers/servo.h"                                 /* サーボ制御API */
#include "ipc/actuator_ipc_server.h"                       /* CPU0-CPU1間IPCサーバーAPI */

/**< 最後に発生したFSPエラー */
EXPORT volatile fsp_err_t g_actuator_service_last_error = FSP_SUCCESS;
/**< アクチュエータ異常フラグ */
EXPORT volatile UH g_actuator_service_fault_flags = ACTUATOR_FAULT_NONE;
EXPORT volatile UW g_actuator_service_applied_sequence;    /**< 最終適用指令sequence */

LOCAL UW g_command_elapsed_ms;                             /**< 最終指令受信からの経過時間 */
LOCAL BOOL g_emergency_stop_latched;                       /**< 緊急停止ラッチ状態 */
LOCAL BOOL g_initialized;                                  /**< アクチュエータ初期化完了状態 */

/** =================================================================*
 * @brief  16 bit値の範囲制限
 * @param[in] value 制限対象の値
 * @param[in] minimum 最小値
 * @param[in] maximum 最大値
 * @param[out] p_limited 制限発生フラグ
 * @return 範囲制限後の値
 * ================================================================= */
LOCAL H clamp_i16(H value, H minimum, H maximum, BOOL * p_limited) {
    if (value < minimum) {
        *p_limited = TRUE;
        return minimum;
    }
    if (value > maximum) {
        *p_limited = TRUE;
        return maximum;
    }

    return value;
}

/** =================================================================*
 * @brief  アクチュエータ安全停止
 * ================================================================= */
LOCAL void actuator_service_safe_stop(void) {
    fsp_err_t const motor_err = drive_service_stop();

    if (FSP_SUCCESS != motor_err) {
        g_actuator_service_last_error = motor_err;
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
    }
    for (UW i = 0U; i < SERVO_COUNT; i++) {
        fsp_err_t const servo_err = servo_disable(i);
        if (FSP_SUCCESS != servo_err) {
            g_actuator_service_last_error = servo_err;
            g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
        }
    }
}

/** =================================================================*
 * @brief  アクチュエータ指令適用
 * @param[in] p_received CPU0から受信した指令
 * ================================================================= */
LOCAL void actuator_service_apply_command(const actuator_command_t * p_received) {
    actuator_command_t command = *p_received;
    BOOL limited = FALSE;

    command.left_target_rpm = clamp_i16(command.left_target_rpm, -DRIVE_TARGET_RPM_MAX, DRIVE_TARGET_RPM_MAX, &limited);
    command.right_target_rpm =
        clamp_i16(command.right_target_rpm, -DRIVE_TARGET_RPM_MAX, DRIVE_TARGET_RPM_MAX, &limited);
    for (UW i = 0U; i < SERVO_COUNT; i++) {
        command.servo_target_deg[i] =
            clamp_i16(command.servo_target_deg[i], STEERING_MIN_DEG, STEERING_MAX_DEG, &limited);
    }
    g_actuator_service_fault_flags &= (UH) ~(ACTUATOR_FAULT_COMMAND_TIMEOUT | ACTUATOR_FAULT_COMMAND_LIMITED);
    if (limited) {
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_COMMAND_LIMITED;
    }

    g_command_elapsed_ms = 0U;
    g_actuator_service_applied_sequence = command.sequence_number;

    if (0U != command.emergency_stop) {
        g_emergency_stop_latched = TRUE;
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE;
        actuator_service_safe_stop();
        return;
    }

    /* ラッチした緊急停止は、アクチュエータ無効指令を先に受けて解除する。 */
    if (g_emergency_stop_latched) {
        actuator_service_safe_stop();
        if (0U == command.actuator_enable) {
            g_emergency_stop_latched = FALSE;
            g_actuator_service_fault_flags &= (UH) ~ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE;
        }
        return;
    }

    if (0U == command.actuator_enable) {
        actuator_service_safe_stop();
        return;
    }

    for (UW i = 0U; i < SERVO_COUNT; i++) {
        fsp_err_t const err = servo_set_target_deg(i, command.servo_target_deg[i]);
        if (FSP_SUCCESS != err) {
            g_actuator_service_last_error = err;
            g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
            actuator_service_safe_stop();
            return;
        }
    }

    fsp_err_t err = drive_service_set_target_rpm(command.left_target_rpm, command.right_target_rpm);
    if (FSP_SUCCESS != err) {
        g_actuator_service_last_error = err;
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
        actuator_service_safe_stop();
    }
}

/** =================================================================*
 * @brief  CPU1アクチュエータアプリケーション初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_service_init(void) {
    g_initialized = FALSE;
    g_command_elapsed_ms = 0U;
    g_emergency_stop_latched = FALSE;
    g_actuator_service_last_error = FSP_SUCCESS;
    g_actuator_service_fault_flags = ACTUATOR_FAULT_NONE;
    g_actuator_service_applied_sequence = 0U;

    fsp_err_t err = drive_service_init();
    if (FSP_SUCCESS == err) {
        err = encoder_init();
    }
    if (FSP_SUCCESS == err) {
        err = servo_init();
    }
    if (FSP_SUCCESS == err) {
        err = actuator_ipc_server_init();
    }

    if (FSP_SUCCESS != err) {
        g_actuator_service_last_error = err;
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
        actuator_service_safe_stop();
        return err;
    }

    /* 完全な有効指令を受信するまで出力を無効にする。 */
    actuator_service_safe_stop();
    g_initialized = TRUE;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  CPU1アクチュエータ1 ms周期処理
 * ================================================================= */
EXPORT void actuator_service_update_1ms(void) {
    if (!g_initialized) {
        actuator_service_safe_stop();
        return;
    }

    encoder_housekeeping_1ms();

    fsp_err_t const motor_err = drive_service_update_1ms();
    if (FSP_SUCCESS != motor_err) {
        g_actuator_service_last_error = motor_err;
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
        actuator_service_safe_stop();
        return;
    }

    if (actuator_ipc_server_take_rx_fault()) {
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_IPC_RX;
    }

    actuator_command_t command;
    if (actuator_ipc_server_take_command(&command)) {
        actuator_service_apply_command(&command);
    } else if (g_command_elapsed_ms < UINT32_MAX) {
        g_command_elapsed_ms++;
    }

    if ((g_command_elapsed_ms >= ACTUATOR_COMMAND_TIMEOUT_MS) &&
        (0U == (g_actuator_service_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT))) {
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_COMMAND_TIMEOUT;
        actuator_service_safe_stop();
    }
}

/** =================================================================*
 * @brief  CPU1アクチュエータ安全停止
 * ================================================================= */
EXPORT void actuator_service_shutdown(void) {
    g_initialized = FALSE;
    actuator_service_safe_stop();
}

/** =================================================================*
 * @brief  CPU1実出力状態取得
 * @param[out] p_status 状態格納先
 * ================================================================= */
EXPORT void actuator_service_status_get(actuator_status_t * p_status) {
    if (NULL == p_status) {
        return;
    }

    p_status->left_duty_permille = g_drive_left_duty_permille;
    p_status->right_duty_permille = g_drive_right_duty_permille;
    p_status->left_encoder_rpm_x10 = (H) g_encoder_left_rpm_x10;
    p_status->right_encoder_rpm_x10 = (H) g_encoder_right_rpm_x10;
    p_status->fault_flags = g_actuator_service_fault_flags;
    p_status->applied_command_sequence = g_actuator_service_applied_sequence;
}
