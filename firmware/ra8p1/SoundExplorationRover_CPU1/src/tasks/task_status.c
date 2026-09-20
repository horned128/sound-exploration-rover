/** =================================================================*
 * @file   task_status.c
 * @brief  CPU1状態表示・実時間周期送信
 * ================================================================= */
#include "task_status.h"                                    /* 状態表示タスクAPI */
#include "config/pin_config.h"                              /* 状態LEDの役割対応 */
#include "config/task_config.h"                             /* 状態タスク周期と優先度 */
#include "services/actuator_service.h"                      /* アクチュエータ異常状態 */
#include "ipc/actuator_ipc_server.h"                        /* CPU1実出力状態IPC送信 */
#include "hal_data.h"                                       /* BSP LED、I/OポートAPI */

IMPORT bsp_leds_t g_bsp_leds;                               /**< BSPが管理するLED構成情報 */

LOCAL void task_status_entry(INT start_code, void * p_extended_information); /* タスク本体 */
LOCAL void task_status_led_write(bsp_io_level_t level);     /* 状態LED出力 */

#define STATUS_TICK_FLAG                   (1U)             /**< 状態通知タスクの周期イベントビット */

LOCAL void task_status_tick(void * p_extended_information); /* 周期通知 */
LOCAL BOOL task_status_time_get(UD * p_now_ms);             /* 単調増加時刻取得 */
LOCAL ID status_flag_id;                                    /**< 周期通知フラグID */
LOCAL ID status_cycle_id;                                   /**< 周期ハンドラID */
EXPORT volatile ER g_task_status_last_error;                /**< カーネルAPI異常 */
EXPORT volatile UW g_task_status_update_count;              /**< 正の経過時間の更新回数 */
EXPORT volatile UW g_task_status_period_last_ms;            /**< 直近更新間隔[ms] */
EXPORT volatile UW g_task_status_period_min_ms;             /**< 最小更新間隔[ms] */
EXPORT volatile UW g_task_status_period_max_ms;             /**< 最大更新間隔[ms] */
EXPORT volatile UW g_task_status_late_count;                /**< 10 ms超の更新回数 */
EXPORT volatile UD g_task_status_elapsed_total_ms;          /**< 実経過時間[ms] */
EXPORT volatile UW g_task_status_snapshot_count;            /**< 状態採取回数 */
EXPORT volatile UW g_task_status_snapshot_period_last_ms;   /**< 状態採取間隔[ms] */
EXPORT volatile UW g_task_status_telemetry_count;           /**< 全語送信完了回数 */
EXPORT volatile UW g_task_status_send_retry_count;          /**< 状態語の送信失敗回数 */
LOCAL ID status_task_id;                                    /**< 状態表示タスクID */
/**< 状態表示周期タスク設定 */
LOCAL T_CTSK const status_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = task_status_entry,
    .itskpri = CPU1_STATUS_TASK_PRIORITY,
    .stksz = CPU1_STATUS_TASK_STACK_SIZE,
};

/**< 通知は合流し、遅延後も最新状態だけを処理する。 */
LOCAL T_CFLG const status_flag_config = {
    .exinf = NULL,
    .flgatr = TA_TFIFO,
    .iflgptn = 0U,
};
/**< 初回はカーネルの+1 tick、その後は絶対周期10 ms。 */
LOCAL T_CCYC const status_cycle_config = {
    .exinf = NULL,
    .cycatr = TA_HLNG,
    .cychdr = task_status_tick,
    .cyctim = CPU1_STATUS_TASK_PERIOD_MS,
    .cycphs = CPU1_STATUS_TASK_PERIOD_MS,
};

/** =================================================================*
 * @brief  状態表示タスク生成
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_status_create(void) {
    g_task_status_last_error = E_OK;
    g_task_status_update_count = 0U;
    g_task_status_period_last_ms = 0U;
    g_task_status_period_min_ms = UINT32_MAX;
    g_task_status_period_max_ms = 0U;
    g_task_status_late_count = 0U;
    g_task_status_elapsed_total_ms = 0U;
    g_task_status_snapshot_count = 0U;
    g_task_status_snapshot_period_last_ms = 0U;
    g_task_status_telemetry_count = 0U;
    g_task_status_send_retry_count = 0U;
    ID const flag_id = tk_cre_flg(&status_flag_config);
    if (flag_id <= 0) {
        g_task_status_last_error = flag_id;
        task_status_delete();
        return APP_FAULT_STATUS_TASK_CREATE;
    }
    status_flag_id = flag_id;
    ID const cycle_id = tk_cre_cyc(&status_cycle_config);
    if (cycle_id <= 0) {
        g_task_status_last_error = cycle_id;
        task_status_delete();
        return APP_FAULT_STATUS_TASK_CREATE;
    }
    status_cycle_id = cycle_id;
    ID const task_id = tk_cre_tsk(&status_task_config);
    if (task_id <= 0) {
        g_task_status_last_error = task_id;
        task_status_delete();
        return APP_FAULT_STATUS_TASK_CREATE;
    }
    status_task_id = task_id;

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  状態表示タスク開始
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_status_start(void) {
    /* タスクが起床直後に異常終了しても、その後に通知を再開しない。 */
    ER err = tk_sta_cyc(status_cycle_id);
    if (E_OK == err) {
        err = tk_sta_tsk(status_task_id, 0);
    }
    if (E_OK != err) {
        g_task_status_last_error = err;
        task_status_delete();
        return APP_FAULT_STATUS_TASK_START;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  状態表示タスク解放
 * ================================================================= */
EXPORT void task_status_delete(void) {
    if (status_cycle_id > 0) {
        (void) tk_stp_cyc(status_cycle_id);
        (void) tk_del_cyc(status_cycle_id);
        status_cycle_id = 0;
    }
    if (status_task_id > 0) {
        (void) tk_ter_tsk(status_task_id);
        (void) tk_del_tsk(status_task_id);
        status_task_id = 0;
    }
    if (status_flag_id > 0) {
        (void) tk_del_flg(status_flag_id);
        status_flag_id = 0;
    }
    task_status_led_write(BSP_IO_LEVEL_HIGH);
}

/** =================================================================*
 * @brief  状態タスク周期通知
 * @param[in] p_extended_information 拡張情報
 * ================================================================= */
LOCAL void task_status_tick(void * p_extended_information) {
    (void) p_extended_information;
    (void) tk_set_flg(status_flag_id, STATUS_TICK_FLAG);
}

/** =================================================================*
 * @brief  状態タスク用単調増加時刻取得
 * @param[out] p_now_ms 稼働時間[ms]
 * @return 取得成功ならTRUE
 * ================================================================= */
LOCAL BOOL task_status_time_get(UD * p_now_ms) {
    SYSTIM now;
    ER const err = tk_get_otm(&now);
    if (E_OK != err) {
        g_task_status_last_error = err;
        return FALSE;
    }
    *p_now_ms = ((UD) (UW) now.hi << 32U) | now.lo;
    return TRUE;
}

/** =================================================================*
 * @brief  CPU1状態表示
 * @details 正常時は500 ms、異常時は50 msごとに赤LEDを反転する。
 * ================================================================= */
LOCAL void task_status_entry(INT start_code, void * p_extended_information) {
    (void) start_code;
    (void) p_extended_information;

    UD previous_ms;
    UD blink_elapsed_ms = 0U;
    UD telemetry_elapsed_ms = 0U;
    UD snapshot_previous_ms = 0U;
    UW telemetry_sequence = 0U;
    UB telemetry_word_index = ACTUATOR_IPC_STATUS_WORD_COUNT;
    actuator_status_t telemetry_status = {0};
    bsp_io_level_t level = BSP_IO_LEVEL_HIGH;

    if (task_status_time_get(&previous_ms)) {
        snapshot_previous_ms = previous_ms;
        while (1) {
            UINT pattern;
            ER const err = tk_wai_flg(status_flag_id, STATUS_TICK_FLAG, TWF_ORW | TWF_BITCLR,
                                     &pattern, TMO_FEVR);
            if (E_OK != err) {
                g_task_status_last_error = err;
                break;
            }
            UD now_ms;
            if (!task_status_time_get(&now_ms)) {
                break;
            }
            if (now_ms < previous_ms) {
                g_task_status_last_error = E_SYS;
                break;
            }
            UD const elapsed = now_ms - previous_ms;
            if (0U == elapsed) {
                continue;
            }
            previous_ms = now_ms;
            UW const elapsed_ms = (elapsed > UINT32_MAX) ? UINT32_MAX : (UW) elapsed;
            g_task_status_period_last_ms = elapsed_ms;
            if (elapsed_ms < g_task_status_period_min_ms) {
                g_task_status_period_min_ms = elapsed_ms;
            }
            if (elapsed_ms > g_task_status_period_max_ms) {
                g_task_status_period_max_ms = elapsed_ms;
            }
            if (elapsed > CPU1_STATUS_TASK_PERIOD_MS) {
                g_task_status_late_count++;
            }
            g_task_status_elapsed_total_ms += elapsed;
            g_task_status_update_count++;

            UW const blink_period_ms = (FSP_SUCCESS == g_actuator_service_last_error)
                ? CPU1_STATUS_HEARTBEAT_PERIOD_MS : CPU1_STATUS_FAULT_BLINK_PERIOD_MS;
            blink_elapsed_ms += elapsed;
            if (blink_elapsed_ms >= blink_period_ms) {
                /* 遅延した反転を連打せず、経過した反転回数の偶奇を反映する。 */
                if (0U != ((blink_elapsed_ms / blink_period_ms) & 1U)) {
                    level = (BSP_IO_LEVEL_LOW == level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
                    task_status_led_write(level);
                }
                blink_elapsed_ms %= blink_period_ms;
            }

            /* 送信中の時間も数え、通常は採取開始から次の採取開始まで100 ms。 */
            telemetry_elapsed_ms += elapsed;
            if ((telemetry_word_index >= ACTUATOR_IPC_STATUS_WORD_COUNT) &&
                (telemetry_elapsed_ms >= CPU1_STATUS_TELEMETRY_PERIOD_MS)) {
                telemetry_status = (actuator_status_t){
                    .sequence_number = telemetry_sequence,
                    .status_uptime_ms = (UW) now_ms,
                };
                actuator_service_status_get(&telemetry_status);
                telemetry_word_index = 0U;
                /* 取り逃した周期をまとめて捨て、古い状態の連続送信を避ける。 */
                telemetry_elapsed_ms %= CPU1_STATUS_TELEMETRY_PERIOD_MS;
                UD const snapshot_elapsed = now_ms - snapshot_previous_ms;
                g_task_status_snapshot_period_last_ms = (snapshot_elapsed > UINT32_MAX)
                    ? UINT32_MAX : (UW) snapshot_elapsed;
                snapshot_previous_ms = now_ms;
                g_task_status_snapshot_count++;
            }
            while (telemetry_word_index < ACTUATOR_IPC_STATUS_WORD_COUNT) {
                /* FIFO満杯時は同じ語を次の起床で再試行し、途中のsnapshotを */
                /* 上書きしない。 */
                fsp_err_t const send_err =
                    actuator_ipc_server_send_status_word(&telemetry_status, telemetry_word_index);
                if (FSP_SUCCESS == send_err) {
                    telemetry_word_index++;
                    if (telemetry_word_index >= ACTUATOR_IPC_STATUS_WORD_COUNT) {
                        telemetry_sequence = (telemetry_sequence + 1U) & ACTUATOR_IPC_SEQUENCE_MASK;
                        g_task_status_telemetry_count++;
                    }
                } else {
                    g_task_status_send_retry_count++;
                    break;
                }
            }
        }
    }
    /* 診断タスクの異常を記録して通知を停止する。駆動出力は */
    /* 1 msタスクの所有を維持する。 */
    (void) tk_stp_cyc(status_cycle_id);
    task_status_led_write(BSP_IO_LEVEL_HIGH);
    tk_ext_tsk();
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
