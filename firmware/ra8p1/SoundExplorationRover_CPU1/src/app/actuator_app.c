/** =================================================================*
 * @file   actuator_app.c
 * @brief  CPU1アクチュエータアプリケーション
 * ================================================================= */
#include "actuator_app.h"                                   /* CPU1アクチュエータアプリケーションAPI */
#include "../cpu1_config.h"                                 /* アクチュエータ動作設定 */
#include "../drivers/dc_motor.h"                            /* DCモーター制御API */
#include "../drivers/encoder.h"                             /* エンコーダ取得API */
#include "../drivers/servo.h"                               /* サーボ制御API */
#include "../ipc/actuator_ipc_server.h"                     /* CPU0-CPU1間IPCサーバーAPI */

volatile fsp_err_t g_actuator_last_error = FSP_SUCCESS;     /**< 最後に発生したFSPエラー */
/**< アクチュエータ異常フラグ */
volatile uint16_t g_actuator_fault_flags = ACTUATOR_FAULT_NONE;

static uint32_t g_command_elapsed_ms;                       /**< 最終指令受信からの経過時間 */
static bool g_emergency_stop_latched;                       /**< 緊急停止ラッチ状態 */
static bool g_initialized;                                  /**< アクチュエータ初期化完了状態 */

/** =================================================================*
 * @brief  16 bit値の範囲制限
 * @param[in] value 制限対象の値
 * @param[in] minimum 最小値
 * @param[in] maximum 最大値
 * @param[out] p_limited 制限発生フラグ
 * @return 範囲制限後の値
 * ================================================================= */
static int16_t clamp_i16(int16_t value, int16_t minimum, int16_t maximum, bool * p_limited) {
    if (value < minimum) {
        *p_limited = true;
        return minimum;
    }
    if (value > maximum) {
        *p_limited = true;
        return maximum;
    }

    return value;
}

/** =================================================================*
 * @brief  アクチュエータ安全停止
 * ================================================================= */
static void actuator_safe_stop(void) {
    fsp_err_t const motor_err = dc_motor_stop();

    if (FSP_SUCCESS != motor_err) {
        g_actuator_last_error = motor_err;
        g_actuator_fault_flags |= ACTUATOR_FAULT_DRIVER;
    }
    for (uint32_t i = 0U; i < SERVO_COUNT; i++) {
        fsp_err_t const servo_err = servo_disable(i);
        if (FSP_SUCCESS != servo_err) {
            g_actuator_last_error = servo_err;
            g_actuator_fault_flags |= ACTUATOR_FAULT_DRIVER;
        }
    }
}

/** =================================================================*
 * @brief  アクチュエータ指令適用
 * @param[in] p_received CPU0から受信した指令
 * ================================================================= */
static void actuator_apply_command(const actuator_command_t * p_received) {
    actuator_command_t command = *p_received;
    bool limited = false;

    command.left_target_rpm = clamp_i16(command.left_target_rpm,
                                        -JGA25_TARGET_RPM_MAX,
                                        JGA25_TARGET_RPM_MAX,
                                        &limited);
    command.right_target_rpm = clamp_i16(command.right_target_rpm,
                                         -JGA25_TARGET_RPM_MAX,
                                         JGA25_TARGET_RPM_MAX,
                                         &limited);
    for (uint32_t i = 0U; i < SERVO_COUNT; i++) {
        command.servo_target_deg[i] = clamp_i16(command.servo_target_deg[i],
                                                STEERING_MIN_DEG,
                                                STEERING_MAX_DEG,
                                                &limited);
    }
    g_actuator_fault_flags &= (uint16_t) ~(ACTUATOR_FAULT_COMMAND_TIMEOUT |
                                            ACTUATOR_FAULT_COMMAND_LIMITED);
    if (limited) {
        g_actuator_fault_flags |= ACTUATOR_FAULT_COMMAND_LIMITED;
    }

    g_command_elapsed_ms = 0U;

    if (0U != command.emergency_stop) {
        g_emergency_stop_latched = true;
        g_actuator_fault_flags |= ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE;
        actuator_safe_stop();
        return;
    }

    /* ラッチした緊急停止は、アクチュエータ無効指令を先に受けて解除する。 */
    if (g_emergency_stop_latched) {
        actuator_safe_stop();
        if (0U == command.actuator_enable) {
            g_emergency_stop_latched = false;
            g_actuator_fault_flags &= (uint16_t) ~ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE;
        }
        return;
    }

    if (0U == command.actuator_enable) {
        actuator_safe_stop();
        return;
    }

    for (uint32_t i = 0U; i < SERVO_COUNT; i++) {
        fsp_err_t const err = servo_set_target_deg(i, command.servo_target_deg[i]);
        if (FSP_SUCCESS != err) {
            g_actuator_last_error = err;
            g_actuator_fault_flags |= ACTUATOR_FAULT_DRIVER;
            actuator_safe_stop();
            return;
        }
    }

    fsp_err_t err = dc_motor_request_rpm(command.left_target_rpm,
                                         command.right_target_rpm);
    if (FSP_SUCCESS != err) {
        g_actuator_last_error = err;
        g_actuator_fault_flags |= ACTUATOR_FAULT_DRIVER;
        actuator_safe_stop();
    }

}

/** =================================================================*
 * @brief  CPU1アクチュエータアプリケーション初期化
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t actuator_app_init(void) {
    g_initialized = false;
    g_command_elapsed_ms = 0U;
    g_emergency_stop_latched = false;
    g_actuator_last_error = FSP_SUCCESS;
    g_actuator_fault_flags = ACTUATOR_FAULT_NONE;

    fsp_err_t err = dc_motor_init();
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
        g_actuator_last_error = err;
        g_actuator_fault_flags |= ACTUATOR_FAULT_DRIVER;
        actuator_safe_stop();
        return err;
    }

    /* 完全な有効指令を受信するまで出力を無効にする。 */
    actuator_safe_stop();
    g_initialized = true;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  CPU1アクチュエータ1 ms周期処理
 * ================================================================= */
void actuator_app_run_1ms(void) {
    if (!g_initialized) {
        actuator_safe_stop();
        return;
    }

    encoder_housekeeping_1ms();

    fsp_err_t const motor_err = dc_motor_housekeeping_1ms();
    if (FSP_SUCCESS != motor_err) {
        g_actuator_last_error = motor_err;
        g_actuator_fault_flags |= ACTUATOR_FAULT_DRIVER;
        actuator_safe_stop();
        return;
    }

    if (actuator_ipc_server_take_rx_fault()) {
        g_actuator_fault_flags |= ACTUATOR_FAULT_IPC_RX;
    }

    actuator_command_t command;
    if (actuator_ipc_server_take_command(&command)) {
        actuator_apply_command(&command);
    } else if (g_command_elapsed_ms < UINT32_MAX) {
        g_command_elapsed_ms++;
    }

    if ((g_command_elapsed_ms >= ACTUATOR_COMMAND_TIMEOUT_MS) &&
        (0U == (g_actuator_fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT))) {
        g_actuator_fault_flags |= ACTUATOR_FAULT_COMMAND_TIMEOUT;
        actuator_safe_stop();
    }

}
