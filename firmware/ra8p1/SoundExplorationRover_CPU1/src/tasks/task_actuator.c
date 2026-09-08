/** =================================================================*
 * @file   task_actuator.c
 * @brief  CPU1アクチュエータ周期タスク
 * ================================================================= */
#include "task_actuator.h"                                    /* アクチュエータタスクAPI */
#include "config/task_config.h"                            /* タスク周期、優先度、スタックサイズ */
#include "services/actuator_service.h"                     /* アクチュエータサービスAPI */

LOCAL void task_actuator_entry(INT start_code, void * p_extended_information); /* タスク本体 */

LOCAL ID actuator_task_id;                                   /**< アクチュエータタスクID */
/**< アクチュエータ周期タスク設定 */
LOCAL T_CTSK const actuator_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = task_actuator_entry,
    .itskpri = CPU1_ACTUATOR_TASK_PRIORITY,
    .stksz = CPU1_ACTUATOR_TASK_STACK_SIZE,
};

/** =================================================================*
 * @brief  アクチュエータ初期化・タスク生成
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_actuator_create(void) {
    if (FSP_SUCCESS != actuator_service_init()) {
        return APP_FAULT_ACTUATOR_INIT;
    }

    actuator_task_id = tk_cre_tsk(&actuator_task_config);
    if (actuator_task_id <= 0) {
        actuator_task_id = 0;
        actuator_service_shutdown();
        return APP_FAULT_ACTUATOR_TASK_CREATE;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  アクチュエータタスク開始
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_actuator_start(void) {
    if (E_OK != tk_sta_tsk(actuator_task_id, 0)) {
        return APP_FAULT_ACTUATOR_TASK_START;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  アクチュエータタスク解放
 * ================================================================= */
EXPORT void task_actuator_delete(void) {
    if (actuator_task_id > 0) {
        (void) tk_ter_tsk(actuator_task_id);
        actuator_service_shutdown();
        (void) tk_del_tsk(actuator_task_id);
        actuator_task_id = 0;
    }
}

/** =================================================================*
 * @brief  アクチュエータ1 ms周期処理
 * @param[in] start_code タスク開始コード
 * @param[in] p_extended_information 拡張情報
 * ================================================================= */
LOCAL void task_actuator_entry(INT start_code, void * p_extended_information) {
    (void) start_code;
    (void) p_extended_information;

    while (1) {
        actuator_service_update_1ms();
        (void) tk_dly_tsk(ACTUATOR_LOOP_PERIOD_MS);
    }
}
