/** =================================================================*
 * @file   task_actuator.c
 * @brief  CPU1アクチュエータの絶対周期駆動
 * ================================================================= */
#include "task_actuator.h"                                  /* アクチュエータタスクAPI */
#include "config/task_config.h"                             /* 周期、優先度、スタックサイズ */
#include "services/actuator_service.h"                      /* 実経過時間による駆動更新 */

#define ACTUATOR_TICK_FLAG                 (1U)

LOCAL void task_actuator_entry(INT start_code, void * p_extended_information); /* タスク本体 */
LOCAL void task_actuator_tick(void * p_extended_information); /* 周期通知 */
LOCAL BOOL task_actuator_time_get(UD * p_now_ms);            /* 単調増加時刻の取得 */

LOCAL ID actuator_task_id;                                  /**< アクチュエータタスクID */
LOCAL ID actuator_flag_id;                                  /**< 周期通知イベントフラグID */
LOCAL ID actuator_cycle_id;                                 /**< 周期ハンドラID */
EXPORT volatile ER g_task_actuator_last_error;              /**< カーネルAPIの最終異常 */
EXPORT volatile UW g_task_actuator_update_count;            /**< 実時間更新の実行回数 */
EXPORT volatile UW g_task_actuator_period_last_ms;          /**< 直近の実更新間隔[ms] */
EXPORT volatile UW g_task_actuator_period_min_ms;           /**< 最小実更新間隔[ms] */
EXPORT volatile UW g_task_actuator_period_max_ms;           /**< 最大実更新間隔[ms] */
EXPORT volatile UW g_task_actuator_late_count;               /**< 1 msを超えた更新間隔の回数 */
EXPORT volatile UD g_task_actuator_elapsed_total_ms;        /**< 開始からの実経過時間[ms] */

/**< 待機はフラグで行い、FSP操作はタスク文脈に限定する。 */
LOCAL T_CTSK const actuator_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = task_actuator_entry,
    .itskpri = CPU1_ACTUATOR_TASK_PRIORITY,
    .stksz = CPU1_ACTUATOR_TASK_STACK_SIZE,
};
/**< 周期通知は蓄積せず、遅延時は最新の時刻との差を1回だけ適用する。 */
LOCAL T_CFLG const actuator_flag_config = {
    .exinf = NULL,
    .flgatr = TA_TFIFO,
    .iflgptn = 0U,
};
/**< 初回起動のみカーネルの+1 tickが入るが、以降は絶対周期で通知する。 */
LOCAL T_CCYC const actuator_cycle_config = {
    .exinf = NULL,
    .cycatr = TA_HLNG,
    .cychdr = task_actuator_tick,
    .cyctim = ACTUATOR_LOOP_PERIOD_MS,
    .cycphs = ACTUATOR_LOOP_PERIOD_MS,
};

/** =================================================================*
 * @brief  アクチュエータ初期化・周期資源生成
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_actuator_create(void) {
    g_task_actuator_last_error = E_OK;
    g_task_actuator_update_count = 0U;
    g_task_actuator_period_last_ms = 0U;
    g_task_actuator_period_min_ms = UINT32_MAX;
    g_task_actuator_period_max_ms = 0U;
    g_task_actuator_late_count = 0U;
    g_task_actuator_elapsed_total_ms = 0U;
    if (FSP_SUCCESS != actuator_service_init()) {
        return APP_FAULT_ACTUATOR_INIT;
    }

    ID const flag_id = tk_cre_flg(&actuator_flag_config);
    if (flag_id <= 0) {
        g_task_actuator_last_error = flag_id;
        task_actuator_delete();
        return APP_FAULT_ACTUATOR_TASK_CREATE;
    }
    actuator_flag_id = flag_id;
    ID const cycle_id = tk_cre_cyc(&actuator_cycle_config);
    if (cycle_id <= 0) {
        g_task_actuator_last_error = cycle_id;
        task_actuator_delete();
        return APP_FAULT_ACTUATOR_TASK_CREATE;
    }
    actuator_cycle_id = cycle_id;
    ID const task_id = tk_cre_tsk(&actuator_task_config);
    if (task_id <= 0) {
        g_task_actuator_last_error = task_id;
        task_actuator_delete();
        return APP_FAULT_ACTUATOR_TASK_CREATE;
    }
    actuator_task_id = task_id;
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  アクチュエータタスクと周期通知の開始
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_actuator_start(void) {
    /* 起床直後のタスクが異常終了した場合も、通知停止を後から取り消さない順序。 */
    ER err = tk_sta_cyc(actuator_cycle_id);
    if (E_OK == err) {
        err = tk_sta_tsk(actuator_task_id, 0);
    }
    if (E_OK != err) {
        g_task_actuator_last_error = err;
        task_actuator_delete();
        return APP_FAULT_ACTUATOR_TASK_START;
    }
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  アクチュエータ周期資源の解放
 * @details 通知元を先に停止し、出力停止後にフラグを解放する。途中生成失敗でも呼出し可能。
 * ================================================================= */
EXPORT void task_actuator_delete(void) {
    if (actuator_cycle_id > 0) {
        (void) tk_stp_cyc(actuator_cycle_id);
        (void) tk_del_cyc(actuator_cycle_id);
        actuator_cycle_id = 0;
    }
    if (actuator_task_id > 0) {
        (void) tk_ter_tsk(actuator_task_id);
        (void) tk_del_tsk(actuator_task_id);
        actuator_task_id = 0;
    }
    actuator_service_shutdown();
    if (actuator_flag_id > 0) {
        (void) tk_del_flg(actuator_flag_id);
        actuator_flag_id = 0;
    }
}

/** =================================================================*
 * @brief  周期通知
 * @param[in] p_extended_information 拡張情報
 * ================================================================= */
LOCAL void task_actuator_tick(void * p_extended_information) {
    (void) p_extended_information;
    (void) tk_set_flg(actuator_flag_id, ACTUATOR_TICK_FLAG);
}

/** =================================================================*
 * @brief  単調増加時刻の取得
 * @param[out] p_now_ms 64 bitの稼働時間[ms]
 * @return 取得成功ならTRUE
 * ================================================================= */
LOCAL BOOL task_actuator_time_get(UD * p_now_ms) {
    SYSTIM now;
    ER const err = tk_get_otm(&now);
    if (E_OK != err) {
        g_task_actuator_last_error = err;
        return FALSE;
    }
    *p_now_ms = ((UD) (UW) now.hi << 32U) | now.lo;
    return TRUE;
}

/** =================================================================*
 * @brief  アクチュエータの実時間周期処理
 * @param[in] start_code タスク開始コード
 * @param[in] p_extended_information 拡張情報
 * ================================================================= */
LOCAL void task_actuator_entry(INT start_code, void * p_extended_information) {
    (void) start_code;
    (void) p_extended_information;
    UD previous_ms;
    if (task_actuator_time_get(&previous_ms)) {
        /* 初期化中の経過時間を速度窓へ含めない。 */
        actuator_service_update(0U);
        while (1) {
            UINT pattern;
            ER const err = tk_wai_flg(actuator_flag_id, ACTUATOR_TICK_FLAG, TWF_ORW | TWF_BITCLR,
                                     &pattern, TMO_FEVR);
            if (E_OK != err) {
                g_task_actuator_last_error = err;
                break;
            }
            UD now_ms;
            if (!task_actuator_time_get(&now_ms)) {
                break;
            }
            if (now_ms < previous_ms) {
                g_task_actuator_last_error = E_SYS;
                break;
            }
            UD const elapsed = now_ms - previous_ms;
            if (0U == elapsed) {
                continue;
            }
            previous_ms = now_ms;
            UW const elapsed_ms = (elapsed > UINT32_MAX) ? UINT32_MAX : (UW) elapsed;
            g_task_actuator_period_last_ms = elapsed_ms;
            if (elapsed_ms < g_task_actuator_period_min_ms) {
                g_task_actuator_period_min_ms = elapsed_ms;
            }
            if (elapsed_ms > g_task_actuator_period_max_ms) {
                g_task_actuator_period_max_ms = elapsed_ms;
            }
            if (elapsed_ms > ACTUATOR_LOOP_PERIOD_MS) {
                g_task_actuator_late_count++;
            }
            g_task_actuator_elapsed_total_ms += elapsed;
            g_task_actuator_update_count++;
            actuator_service_update(elapsed_ms);
        }
    }
    /* カーネル待機／時刻取得異常時は駆動を止め、周期通知を停止する。 */
    actuator_service_shutdown();
    (void) tk_stp_cyc(actuator_cycle_id);
    tk_ext_tsk();
}
