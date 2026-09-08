/** =================================================================*
 * @file   task_status.c
 * @brief  CPU1状態LED表示タスク
 * ================================================================= */
#include "task_status.h"                                      /* 状態表示タスクAPI */
#include "config/pin_config.h"                             /* 状態LEDの役割対応 */
#include "config/task_config.h"                            /* 状態タスク周期と優先度 */
#include "services/actuator_service.h"                     /* アクチュエータ異常状態 */
#include "ipc/actuator_ipc_server.h"                       /* CPU1実出力状態IPC送信 */
#include "hal_data.h"                                       /* BSP LED、I/OポートAPI */

IMPORT bsp_leds_t g_bsp_leds;                               /**< BSPが管理するLED構成情報 */

LOCAL void task_status_entry(INT start_code, void * p_extended_information); /* タスク本体 */
LOCAL void task_status_led_write(bsp_io_level_t level);     /* 状態LED出力 */

LOCAL ID status_task_id;                                     /**< 状態表示タスクID */
/**< 状態表示周期タスク設定 */
LOCAL T_CTSK const status_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = task_status_entry,
    .itskpri = CPU1_STATUS_TASK_PRIORITY,
    .stksz = CPU1_STATUS_TASK_STACK_SIZE,
};

/** =================================================================*
 * @brief  状態表示タスク生成
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_status_create(void) {
    status_task_id = tk_cre_tsk(&status_task_config);
    if (status_task_id <= 0) {
        status_task_id = 0;
        return APP_FAULT_STATUS_TASK_CREATE;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  状態表示タスク開始
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_status_start(void) {
    if (E_OK != tk_sta_tsk(status_task_id, 0)) {
        return APP_FAULT_STATUS_TASK_START;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  状態表示タスク解放
 * ================================================================= */
EXPORT void task_status_delete(void) {
    if (status_task_id > 0) {
        (void) tk_ter_tsk(status_task_id);
        (void) tk_del_tsk(status_task_id);
        status_task_id = 0;
    }
    task_status_led_write(BSP_IO_LEVEL_HIGH);
}

/** =================================================================*
 * @brief  CPU1状態表示
 * @details 正常時は500 ms、異常時は50 msごとに赤LEDを反転する。
 * ================================================================= */
LOCAL void task_status_entry(INT start_code, void * p_extended_information) {
    (void) start_code;
    (void) p_extended_information;

    UW elapsed_ms = 0U;
    UW telemetry_elapsed_ms = 0U;
    UW telemetry_sequence = 0U;
    UB telemetry_word_index = ACTUATOR_IPC_STATUS_WORD_COUNT;
    actuator_status_t telemetry_status = {0};
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;

    while (1) {
        UW const blink_period_ms = (FSP_SUCCESS == g_actuator_service_last_error) ? CPU1_STATUS_HEARTBEAT_PERIOD_MS
                                                                         : CPU1_STATUS_FAULT_BLINK_PERIOD_MS;

        elapsed_ms += CPU1_STATUS_TASK_PERIOD_MS;
        if (telemetry_word_index >= ACTUATOR_IPC_STATUS_WORD_COUNT) {
            telemetry_elapsed_ms += CPU1_STATUS_TASK_PERIOD_MS;
        }
        if (elapsed_ms >= blink_period_ms) {
            task_status_led_write(level);
            level = (BSP_IO_LEVEL_LOW == level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
            elapsed_ms = 0U;
        }

        if ((telemetry_word_index >= ACTUATOR_IPC_STATUS_WORD_COUNT) &&
            (telemetry_elapsed_ms >= CPU1_STATUS_TELEMETRY_PERIOD_MS)) {
            telemetry_status = (actuator_status_t){.sequence_number = telemetry_sequence};
            actuator_service_status_get(&telemetry_status);
            telemetry_word_index = 0U;
            telemetry_elapsed_ms = 0U;
        }
        if ((telemetry_word_index < ACTUATOR_IPC_STATUS_WORD_COUNT) &&
            (FSP_SUCCESS == actuator_ipc_server_send_status_word(&telemetry_status, telemetry_word_index))) {
            telemetry_word_index++;
            if (telemetry_word_index >= ACTUATOR_IPC_STATUS_WORD_COUNT) {
                telemetry_sequence = (telemetry_sequence + 1U) & ACTUATOR_IPC_SEQUENCE_MASK;
            }
        }

        (void) tk_dly_tsk(CPU1_STATUS_TASK_PERIOD_MS);
    }
}

/** =================================================================*
 * @brief  CPU1起動異常表示・停止
 * @param[in] fault CPU1異常コード
 * ================================================================= */
EXPORT void task_status_halt(app_fault_t fault) {
    actuator_service_shutdown();

    while (1) {
        for (INT pulse = 0; pulse < (INT) fault; pulse++) {
            task_status_led_write(BSP_IO_LEVEL_LOW);
            (void) tk_dly_tsk(100U);
            task_status_led_write(BSP_IO_LEVEL_HIGH);
            (void) tk_dly_tsk(100U);
        }
        (void) tk_dly_tsk(1000U);
    }
}

/** =================================================================*
 * @brief  CPU1状態LED出力
 * @param[in] level 出力レベル
 * ================================================================= */
LOCAL void task_status_led_write(bsp_io_level_t level) {
    bsp_leds_t const leds = g_bsp_leds;
    if (leds.led_count > CPU1_STATUS_LED_INDEX) {
        (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, leds.p_leds[CPU1_STATUS_LED_INDEX], level);
    }
}
