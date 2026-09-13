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

#if DRIVE_MEASUREMENT_TEST_ENABLE
EXPORT volatile UB g_drive_measurement_mode = DRIVE_MEASUREMENT_MODE_NORMAL;
EXPORT volatile UH g_drive_measurement_duty_permille = 0U;
EXPORT volatile UH g_drive_measurement_applied_duty_permille = 0U;
EXPORT volatile UB g_drive_measurement_status = DRIVE_MEASUREMENT_STATUS_DISABLED;
EXPORT volatile BOOL g_drive_measurement_output_authorized = FALSE;
EXPORT volatile W g_drive_measurement_stop_count_left = 0;
EXPORT volatile W g_drive_measurement_stop_count_right = 0;
EXPORT volatile BOOL g_drive_measurement_stop_capture_valid = FALSE;
#endif

LOCAL UW g_command_elapsed_ms;                             /**< 最終指令受信からの経過時間 */
LOCAL BOOL g_emergency_stop_latched;                       /**< 緊急停止ラッチ状態 */
LOCAL BOOL g_initialized;                                  /**< アクチュエータ初期化完了状態 */
LOCAL BOOL g_actuator_output_enabled;                      /**< 有効な通常指令を受信済み */
#if DRIVE_MEASUREMENT_TEST_ENABLE
LOCAL UB g_drive_measurement_previous_mode;                /**< 前回適用した計測モード */
#endif

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
    g_actuator_output_enabled = FALSE;

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

#if DRIVE_MEASUREMENT_TEST_ENABLE
/** =================================================================*
 * @brief  実機計測用の一時出力モードを適用
 * @details Live Watchから直接PWM変数を変更せず、通常の駆動サービスを通す。
 *          CPU0指令がtimeoutまたは異常で無効になった場合は計測出力も許可しない。
 * ================================================================= */
LOCAL void actuator_service_apply_measurement_test(void) {
    UB const mode = g_drive_measurement_mode;
    UB const previous_mode = g_drive_measurement_previous_mode;
    g_drive_measurement_previous_mode = mode;
    g_drive_measurement_output_authorized = FALSE;
    g_drive_measurement_applied_duty_permille = 0U;

    if ((DRIVE_MEASUREMENT_MODE_DUTY_OVERRIDE == mode) &&
        (DRIVE_MEASUREMENT_MODE_DUTY_OVERRIDE != previous_mode)) {
        g_drive_measurement_stop_capture_valid = FALSE;
    }
    if ((DRIVE_MEASUREMENT_MODE_FORCE_STOP == mode) &&
        (DRIVE_MEASUREMENT_MODE_FORCE_STOP != previous_mode)) {
        /* encoder_housekeeping()直後なので、停止指令適用直前の値を保存できる。 */
        g_drive_measurement_stop_count_left = encoder_left_count_get();
        g_drive_measurement_stop_count_right = encoder_right_count_get();
        g_drive_measurement_stop_capture_valid = TRUE;
    }

    if (DRIVE_MEASUREMENT_MODE_NORMAL == mode) {
        g_drive_measurement_status = DRIVE_MEASUREMENT_STATUS_DISABLED;
        return;
    }

    if (DRIVE_MEASUREMENT_MODE_FORCE_STOP == mode) {
        fsp_err_t const err = drive_service_stop();
        g_actuator_output_enabled = FALSE;
        g_drive_measurement_status =
            (FSP_SUCCESS == err) ? DRIVE_MEASUREMENT_STATUS_FORCE_STOP : DRIVE_MEASUREMENT_STATUS_DRIVER_ERROR;
        if (FSP_SUCCESS != err) {
            g_actuator_service_last_error = err;
            g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
        }
        return;
    }

    if (DRIVE_MEASUREMENT_MODE_DUTY_OVERRIDE != mode) {
        (void) drive_service_stop();
        g_actuator_output_enabled = FALSE;
        g_drive_measurement_status = DRIVE_MEASUREMENT_STATUS_INVALID;
        return;
    }

    if (!g_actuator_output_enabled ||
        (0U != (g_actuator_service_fault_flags &
                (ACTUATOR_FAULT_COMMAND_TIMEOUT | ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE | ACTUATOR_FAULT_DRIVER)))) {
        (void) drive_service_stop();
        g_drive_measurement_status = DRIVE_MEASUREMENT_STATUS_WAITING_FOR_ENABLE;
        return;
    }

    UH duty_permille = g_drive_measurement_duty_permille;
    if (duty_permille > DRIVE_MEASUREMENT_MAX_DUTY_PERMILLE) {
        duty_permille = DRIVE_MEASUREMENT_MAX_DUTY_PERMILLE;
    }
    fsp_err_t const err = drive_service_set_test_duty_permille(duty_permille);
    if (FSP_SUCCESS != err) {
        g_actuator_service_last_error = err;
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
        (void) drive_service_stop();
        g_drive_measurement_status = DRIVE_MEASUREMENT_STATUS_DRIVER_ERROR;
        return;
    }

    g_drive_measurement_applied_duty_permille = duty_permille;
    g_drive_measurement_output_authorized = TRUE;
    g_drive_measurement_status = DRIVE_MEASUREMENT_STATUS_ACTIVE;
}
#endif

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
    } else {
        g_actuator_output_enabled = TRUE;
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
    g_actuator_output_enabled = FALSE;
#if DRIVE_MEASUREMENT_TEST_ENABLE
    g_drive_measurement_mode = DRIVE_MEASUREMENT_MODE_NORMAL;
    g_drive_measurement_duty_permille = 0U;
    g_drive_measurement_applied_duty_permille = 0U;
    g_drive_measurement_status = DRIVE_MEASUREMENT_STATUS_DISABLED;
    g_drive_measurement_output_authorized = FALSE;
    g_drive_measurement_stop_count_left = 0;
    g_drive_measurement_stop_count_right = 0;
    g_drive_measurement_stop_capture_valid = FALSE;
    g_drive_measurement_previous_mode = DRIVE_MEASUREMENT_MODE_NORMAL;
#endif

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
 * @brief  CPU1アクチュエータ実時間更新
 * @param[in] elapsed_ms 前回更新からの実経過時間[ms]（初回0）
 * ================================================================= */
EXPORT void actuator_service_update(UW elapsed_ms) {
    if (!g_initialized) {
        actuator_service_safe_stop();
        return;
    }

    encoder_housekeeping(elapsed_ms);

    if (actuator_ipc_server_take_rx_fault()) {
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_IPC_RX;
    }
    actuator_command_t command;
    BOOL const received = actuator_ipc_server_take_command(&command);
    if (!received) {
        UW const remaining = UINT32_MAX - g_command_elapsed_ms;
        g_command_elapsed_ms += (elapsed_ms < remaining) ? elapsed_ms : remaining;
    }

    /* 期限切れ後のPWM更新を防ぐ。有効な新着指令は従来どおりtimeoutを更新する。 */
    if (!received && (g_command_elapsed_ms >= ACTUATOR_COMMAND_TIMEOUT_MS) &&
        (0U == (g_actuator_service_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT))) {
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_COMMAND_TIMEOUT;
        actuator_service_safe_stop();
    }

    fsp_err_t const motor_err = drive_service_update(elapsed_ms);
    if (FSP_SUCCESS != motor_err) {
        g_actuator_service_last_error = motor_err;
        g_actuator_service_fault_flags |= ACTUATOR_FAULT_DRIVER;
        actuator_service_safe_stop();
        return;
    }

    /* 過去の経過時間で新着目標をランプさせない。新目標への変化は次回更新から。 */
    if (received) {
        actuator_service_apply_command(&command);
    }
#if DRIVE_MEASUREMENT_TEST_ENABLE
    actuator_service_apply_measurement_test();
#endif
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
