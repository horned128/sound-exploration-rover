/** =================================================================*
 * @file   task_think.c
 * @brief  CPU0自律走行・現場学習タスク
 * ================================================================= */
#include "task_think.h"                                     /* CPU0思考タスクAPI */
#include "config/control_config.h"                          /* 思考モードと走行値 */
#include "config/pin_config.h"                              /* LEDの役割設定 */
#include "config/sensor_config.h"                           /* センサー安全判定値 */
#include "config/task_config.h"                             /* 思考周期と優先度 */
#include "control/control_mlp_planner.h"                     /* TFLM制御MLPプランナ */
#include "control/obstacle_avoidance_controller.h"          /* ToF・IMU走行判断 */
#include "control/safety_arbiter.h"                         /* 安全調停・ToF veto集約 */
#include "control/sensor_liveness.h"                        /* 取得タスクと独立した更新監視 */
#include "control/sound_follow_controller.h"                /* 音源追従状態機械 */
#include <math.h>                                           /* roundf */
#include "hal_data.h"                                       /* BSP LED情報、ピンAPI */
#include "services/acoustic_identifier.h"                   /* 見本leave-one-outしきい値算出 */
#include "services/prototype_storage.h"                     /* Code MRAMプロトタイプ保存 */
#include "task_acoustic_link.h"                             /* 最新音響状態取得API */
#include "task_command.h"                                   /* 最新アクチュエータ目標更新API */
#include "task_infer.h"                                     /* 音響判定結果取得と保存データ更新 */
#include "task_sensor.h"                                    /* 最新I2Cセンサー状態取得API */
#include <string.h>                                         /* 学習バッファ初期化 */

#define CPU0_THINK_EVENT_CLEAR_IPC_SEND    (1UL << 31)      /**< IPC送信異常の回復通知イベントビット */
#define CPU0_THINK_EVENT_MASK              /**< 思考タスクが待つイベントビット全体 */ \
    ((UINT) (APP_FAULT_ALL_MASK | CPU0_THINK_EVENT_CLEAR_IPC_SEND))
#define CPU0_THINK_LEARNING_SAMPLE_COUNT   (5U)             /**< 思考学習サンプルの個数 */

IMPORT bsp_leds_t g_bsp_leds;                               /**< BSPのLED構成情報 */

LOCAL void task_think_entry(INT stacd, void * exinf);       /* 思考タスク本体 */
LOCAL ER task_think_publish_target(const sound_follow_output_t * p_output); /* 追従指令の4輪展開 */
LOCAL ER task_think_publish_motion(H steering_deg, H left_rpm, H right_rpm, BOOL actuator_enable,
                                   BOOL emergency_stop);   /* 共通走行指令の4輪展開 */
/* 音源追従の近接安全判定 */
LOCAL BOOL task_think_sound_motion_allowed(const sensor_snapshot_t * p_snapshot); /* 音源追従走行可否判定 */
LOCAL void task_think_led_write(BOOL blue_on, BOOL green_on); /* 2LED一括更新 */
LOCAL void task_think_learning_capture(const task_acoustic_link_snapshot_t * p_snapshot,
                                       BOOL observation_usable); /* 新規特徴量パッチの学習 */

LOCAL UW learning_button_press_ms;                          /**< SW1継続押下時間[ms] */
LOCAL BOOL learning_button_handled;                         /**< 同一押下の多重切替防止 */
LOCAL UW learning_last_feature_generation;                  /**< 最後に収集した特徴量世代 */
LOCAL prototype_storage_data_t storage_data;                /**< 読込済みまたは保存対象プロトタイプ */
LOCAL prototype_storage_data_t storage_candidate;           /**< 保存処理用の作業コピー */
LOCAL UW task_think_fault_code(UW fault_flags);             /* LED表示用異常番号 */
/* 状態LED更新 */
LOCAL void task_think_led_update(UW state_elapsed_ms, UW heartbeat_elapsed_ms, UW fault_elapsed_ms); /* LED更新 */

/**< 他タスクからの異常通知を集約するイベントフラグ設定 */
LOCAL T_CFLG const think_fault_flag_config = {
    .flgatr = TA_TFIFO | TA_WSGL,
    .iflgptn = 0U,
};

/**< 自律走行判断を行う思考タスク設定 */
LOCAL T_CTSK const think_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) task_think_entry,
    .itskpri = CPU0_THINK_TASK_PRIORITY,
    .stksz = CPU0_THINK_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL sensor_liveness_t sensor_liveness;                    /**< 思考側のセンサー更新監視 */
LOCAL UW sensor_now_ms;                                     /**< センサー鮮度判定と共通の実時刻[ms] */
EXPORT volatile UW g_task_think_sensor_watchdog_ms;         /**< 更新進行を観測してからの時間[ms] */
EXPORT volatile BOOL g_task_think_sensor_fresh;             /**< 更新期限内か */
EXPORT volatile ER g_task_think_sensor_clock_error;         /**< 実時間取得の異常 */

/** =================================================================*
 * @brief  センサー取得タスクが止まっても進む時刻で鮮度を確認
 * @param[in] p_snapshot 最新値
 * @param[in] snapshot_error 取得結果
 * @return 更新進行を観測し期限内ならTRUE
 * ================================================================= */
LOCAL BOOL task_think_sensor_fresh(const sensor_snapshot_t * p_snapshot, ER snapshot_error) {
    SYSTIM now;
    ER const err = tk_get_otm(&now);
    UD const now_ms = (E_OK == err) ? (((UD) (UW) now.hi << 32U) | now.lo) : 0U;
    sensor_now_ms = (UW) now_ms;
    g_task_think_sensor_clock_error = err;
    g_task_think_sensor_fresh = sensor_liveness_update(&sensor_liveness, p_snapshot->update_count, now_ms,
        (E_OK == err) && (E_OK == snapshot_error) && p_snapshot->initialized, CPU0_SENSOR_STALE_TIMEOUT_MS);
    g_task_think_sensor_watchdog_ms = sensor_liveness.age_ms;
    return g_task_think_sensor_fresh;
}

LOCAL ID think_task_id;                                     /**< 思考タスクID */
LOCAL ID think_fault_flag_id;                               /**< CPU0異常イベントフラグID */
LOCAL BOOL think_task_started;                              /**< 思考タスク開始状態 */

EXPORT volatile sound_follow_state_t g_task_think_state;    /**< 現在の思考状態 */
EXPORT volatile UW g_task_think_cycle_count;                /**< 思考周期実行回数 */
EXPORT volatile UW g_task_think_observation_sequence;       /**< 最終判断観測sequence */
EXPORT volatile UW g_task_think_observation_watchdog_ms;    /**< 観測更新停止時間 */
EXPORT volatile BOOL g_task_think_link_ready;               /**< 音響リンク判断 */
EXPORT volatile BOOL g_task_think_new_observation;          /**< 新規観測判断 */
EXPORT volatile H g_task_think_steering_deg;                /**< 操舵判断値 */
EXPORT volatile H g_task_think_left_rpm;                    /**< 左RPM判断値 */
EXPORT volatile H g_task_think_right_rpm;                   /**< 右RPM判断値 */
EXPORT volatile BOOL g_task_think_actuator_enable;          /**< 出力許可判断 */
EXPORT volatile BOOL g_task_think_emergency_stop;           /**< 非常停止判断 */
/**< 選択センサー走行ルール */
EXPORT volatile obstacle_avoidance_rule_t g_task_think_sensor_rule;
EXPORT volatile UW g_task_think_fault_flags;                /**< CPU0異常ラッチ */
EXPORT volatile BOOL g_task_think_learning_mode;            /**< 現場学習モード */
EXPORT volatile UB g_task_think_learning_samples;           /**< 収集済み音響見本数 */
EXPORT volatile BOOL g_task_think_storage_valid;            /**< 有効なMRAMプロトタイプ有無 */
/**< 直近MRAM処理結果 */
EXPORT volatile prototype_storage_result_t g_task_think_storage_result;

/** =================================================================*
 * @brief  音源追従で前進してよいToF状態か判定
 * @details センサー取得失敗、更新期限超過、ToF無効、またはいずれかの測距が
 *          ハード停止距離未満なら走行を許可しない。
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 3台のToFが有効かつ近接障害物なしならtrue
 * ================================================================= */
LOCAL BOOL task_think_sound_motion_allowed(const sensor_snapshot_t * p_snapshot) {
    return (BOOL) safety_arbiter_tof_usable(p_snapshot);
}

/** =================================================================*
 * @brief  思考タスクと異常イベント生成
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_think_create(void) {
    sensor_liveness = (sensor_liveness_t){.age_ms = UINT32_MAX};
    g_task_think_sensor_watchdog_ms = UINT32_MAX;
    g_task_think_sensor_fresh = FALSE;
    g_task_think_sensor_clock_error = E_OK;
    think_task_id = 0;
    think_fault_flag_id = 0;
    think_task_started = FALSE;
    g_task_think_state = CPU0_THINK_STATE_WAIT_LINK;
    g_task_think_cycle_count = 0U;
    g_task_think_observation_sequence = 0U;
    g_task_think_observation_watchdog_ms = UINT32_MAX;
    g_task_think_link_ready = FALSE;
    g_task_think_new_observation = FALSE;
    g_task_think_steering_deg = 0;
    g_task_think_left_rpm = 0;
    g_task_think_right_rpm = 0;
    g_task_think_actuator_enable = FALSE;
    g_task_think_emergency_stop = TRUE;
    g_task_think_sensor_rule = CPU0_SENSOR_RULE_SAFE_STOP;
    g_task_think_fault_flags = APP_FAULT_NONE;
    g_task_think_learning_mode = FALSE;
    g_task_think_learning_samples = 0U;
    g_task_think_storage_valid = FALSE;
    g_task_think_storage_result = CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
    sound_follow_controller_init();
#if (CPU0_SENSOR_I2C_ENABLED != 0U)
    obstacle_avoidance_controller_init();
#endif
#if (CPU0_USE_CONTROL_MLP != 0U)
    (void) control_mlp_planner_init();
#endif

    think_fault_flag_id = tk_cre_flg(&think_fault_flag_config);
    if (think_fault_flag_id <= 0) {
        think_fault_flag_id = 0;
        return APP_FAULT_TASK_CREATE;
    }

    think_task_id = tk_cre_tsk(&think_task_config);
    if (think_task_id <= 0) {
        think_task_id = 0;
        task_think_delete();
        return APP_FAULT_TASK_CREATE;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  思考タスク開始
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_think_start(void) {
    if (think_task_id <= 0) {
        return APP_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(think_task_id, 0);
    if (E_OK != err) {
        return APP_FAULT_TASK_START;
    }
    think_task_started = TRUE;
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  思考タスクと異常イベント解放
 * ================================================================= */
EXPORT void task_think_delete(void) {
    if (think_task_id > 0) {
        if (think_task_started) {
            (void) tk_ter_tsk(think_task_id);
        }
        (void) tk_del_tsk(think_task_id);
        think_task_id = 0;
        think_task_started = FALSE;
    }

    if (think_fault_flag_id > 0) {
        (void) tk_del_flg(think_fault_flag_id);
        think_fault_flag_id = 0;
    }

#if (CPU0_USE_CONTROL_MLP != 0U)
    control_mlp_planner_reset();
#endif
}

/** =================================================================*
 * @brief  他タスクから思考タスクへ異常通知
 * @param[in] fault CPU0異常ビット
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER task_think_report_fault(app_fault_t fault) {
    if (APP_FAULT_NONE == fault) {
        return E_OK;
    }
    if (think_fault_flag_id <= 0) {
        return E_NOEXS;
    }

    return tk_set_flg(think_fault_flag_id, (UINT) fault);
}

/** =================================================================*
 * @brief  最新の96次元要約を1回だけ現場見本として取り込む
 * @details 能動フレーム不足は判定不能なので、零ベクトルを見本として保存しない。
 * ================================================================= */
LOCAL void task_think_learning_capture(const task_acoustic_link_snapshot_t * p_snapshot,
                                       BOOL observation_usable) {
    if (!g_task_think_learning_mode ||
        (g_task_think_learning_samples >= CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT)) {
        return;
    }
    if ((NULL == p_snapshot) || (!observation_usable) ||
        (p_snapshot->observation.level_dbfs_x100 < CPU0_SOUND_TRIGGER_DBFS_X100) ||
        (0U == p_snapshot->observation.vad)) {
        return;
    }

    /* 同じfeature_generationを二重保存しないための推論結果保持領域。 */
    LOCAL task_infer_result_t result;
    if ((E_OK != task_infer_result_get(&result)) || (result.feature_generation == learning_last_feature_generation)) {
        return;
    }
    learning_last_feature_generation = result.feature_generation;
    if (!result.summary_valid) {
        return;
    }
    memcpy(storage_data.samples[g_task_think_learning_samples], result.summary,
           sizeof(storage_data.samples[g_task_think_learning_samples]));
    g_task_think_learning_samples++;
    storage_data.sample_count = g_task_think_learning_samples;
}

/** =================================================================*
 * @brief  回復を確認した一過性異常の解除通知
 * @details 現在はCPU1との双方向IPCおよび安全指令反映を確認できる
 *          APP_FAULT_IPC_SENDだけを解除対象とする。
 * @param[in] fault 解除するCPU0異常ビット
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER task_think_clear_fault(app_fault_t fault) {
    if (APP_FAULT_IPC_SEND != fault) {
        return E_PAR;
    }
    if (think_fault_flag_id <= 0) {
        return E_NOEXS;
    }

    return tk_set_flg(think_fault_flag_id, (UINT) CPU0_THINK_EVENT_CLEAR_IPC_SEND);
}

/** =================================================================*
 * @brief  追従指令の4輪展開
 * @details 前後輪を逆相操舵し、左右DCモーターを同じ更新で指令する。
 * @param[in] p_output 音源追従状態機械の出力
 * @return μT-Kernelエラーコード
 * ================================================================= */
LOCAL ER task_think_publish_target(const sound_follow_output_t * p_output) {
    if (NULL == p_output) {
        return E_PAR;
    }

    return task_think_publish_motion(p_output->steering_deg, p_output->left_rpm, p_output->right_rpm,
                                     p_output->actuator_enable, p_output->emergency_stop);
}

/** =================================================================*
 * @brief  共通走行指令を4輪操舵・左右DCモーターの目標へ展開
 * @details 前後輪を逆相操舵し、左右DCモーターを同じ更新で指令する。
 * @param[in] steering_deg 右正の車体操舵角
 * @param[in] left_rpm 論理左モーター目標RPM
 * @param[in] right_rpm 論理右モーター目標RPM
 * @param[in] actuator_enable 出力許可
 * @param[in] emergency_stop 非常停止指定
 * @return μT-Kernelエラーコード
 * ================================================================= */
LOCAL ER task_think_publish_motion(H steering_deg, H left_rpm, H right_rpm, BOOL actuator_enable,
                                   BOOL emergency_stop) {

    rover_motion_target_t target = {
        .left_target_rpm = left_rpm,
        .right_target_rpm = right_rpm,
        .actuator_enable = actuator_enable,
        .emergency_stop = emergency_stop,
    };

    H const front_steering_deg = (H) (CPU0_STEERING_SERVO_OUTPUT_SIGN * steering_deg);
    /* FR */
    target.servo_target_deg[0] = front_steering_deg;
    /* FL */
    target.servo_target_deg[1] = front_steering_deg;
    /* RR */
    target.servo_target_deg[2] = (H) -front_steering_deg;
    /* RL */
    target.servo_target_deg[3] = (H) -front_steering_deg;

    return task_command_set_target(&target);
}

/** =================================================================*
 * @brief  青・緑LED一括更新
 * @param[in] blue_on 青LED点灯状態
 * @param[in] green_on 緑LED点灯状態
 * ================================================================= */
LOCAL void task_think_led_write(BOOL blue_on, BOOL green_on) {
    bsp_leds_t const leds = g_bsp_leds;
    if (leds.led_count <= CPU0_THINK_GREEN_LED_INDEX) {
        return;
    }

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[CPU0_THINK_BLUE_LED_INDEX],
                   blue_on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinWrite((bsp_io_port_pin_t) leds.p_leds[CPU0_THINK_GREEN_LED_INDEX],
                   green_on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();
}

/** =================================================================*
 * @brief  異常ビットを青LED点滅回数へ変換
 * @param[in] fault_flags CPU0異常ラッチ
 * @return 1～6の異常番号
 * ================================================================= */
LOCAL UW task_think_fault_code(UW fault_flags) {
    if (0U != (fault_flags & (APP_FAULT_TASK_CREATE | APP_FAULT_TASK_START))) {
        return 1U;
    }
    if (0U != (fault_flags & APP_FAULT_IPC_INIT)) {
        return 2U;
    }
    if (0U != (fault_flags & APP_FAULT_IPC_SEND)) {
        return 3U;
    }
    if (0U != (fault_flags & APP_FAULT_COMMAND_TARGET_TIMEOUT)) {
        return 4U;
    }
    if (0U != (fault_flags & APP_FAULT_USB_INIT)) {
        return 5U;
    }
    return 6U;
}

/** =================================================================*
 * @brief  現在状態を2種類のLEDへ表示
 * @param[in] state_elapsed_ms 現状態の経過時間
 * @param[in] heartbeat_elapsed_ms heartbeat周期内の時刻
 * @param[in] fault_elapsed_ms fault点滅周期内の時刻
 * ================================================================= */
LOCAL void task_think_led_update(UW state_elapsed_ms, UW heartbeat_elapsed_ms, UW fault_elapsed_ms) {
    BOOL blue_on = FALSE;
    BOOL green_on = FALSE;

    if (APP_FAULT_NONE != g_task_think_fault_flags) {
        UW const code = task_think_fault_code(g_task_think_fault_flags);
        UW const pulse_window_ms = code * CPU0_LED_FAULT_PULSE_MS * 2U;
        UW const pattern_ms = pulse_window_ms + CPU0_LED_FAULT_GAP_MS;
        UW const position_ms = fault_elapsed_ms % pattern_ms;

        green_on = TRUE;
        blue_on = (position_ms < pulse_window_ms) && (0U == ((position_ms / CPU0_LED_FAULT_PULSE_MS) & 1U));
    } else {
        green_on = heartbeat_elapsed_ms < CPU0_LED_HEARTBEAT_PULSE_MS;

        switch (g_task_think_state) {
        case CPU0_THINK_STATE_WAIT_LINK:
            green_on = FALSE;
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_WAIT_LINK_BLINK_MS) & 1U);
            break;

        case CPU0_THINK_STATE_LISTEN:
            blue_on = state_elapsed_ms % CPU0_LED_LISTEN_BLINK_MS < CPU0_LED_HEARTBEAT_PULSE_MS;
            break;

        case CPU0_THINK_STATE_STEER_PREP:
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_STEER_BLINK_MS) & 1U);
            break;

        case CPU0_THINK_STATE_MOVE_STEP:
            blue_on = TRUE;
            break;

        case CPU0_THINK_STATE_SETTLE:
        case CPU0_THINK_STATE_COOLDOWN:
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_SETTLE_BLINK_MS) & 1U);
            break;

        case CPU0_THINK_STATE_SENSOR_SAFE_STOP:
        case CPU0_THINK_STATE_SENSOR_BLOCKED_STOP:
        case CPU0_THINK_STATE_SENSOR_IMU_STOP:
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_WAIT_LINK_BLINK_MS) & 1U);
            break;

        case CPU0_THINK_STATE_SENSOR_FORWARD:
            blue_on = TRUE;
            break;

        case CPU0_THINK_STATE_SENSOR_CAUTION_FORWARD:
            blue_on = state_elapsed_ms % CPU0_LED_LISTEN_BLINK_MS < CPU0_LED_HEARTBEAT_PULSE_MS;
            break;

        case CPU0_THINK_STATE_SENSOR_TURN_LEFT:
        case CPU0_THINK_STATE_SENSOR_TURN_RIGHT:
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_STEER_BLINK_MS) & 1U);
            break;

        default:
            break;
        }
    }

    if (g_task_think_learning_mode) {
        blue_on = 0U == ((state_elapsed_ms / 100U) & 1U);
        green_on = !blue_on;
        if (g_task_think_learning_samples > 0U) {
            if (state_elapsed_ms % 500U < 100U) {
                blue_on = TRUE;
                green_on = TRUE;
            }
        }
    }
    task_think_led_write(blue_on, green_on);
}

/** =================================================================*
 * @brief  思考タスク本体
 * ================================================================= */
LOCAL void task_think_entry(INT stacd, void * exinf) {
    g_task_think_storage_result = prototype_storage_init();
    if (CPU0_PROTOTYPE_STORAGE_OK == g_task_think_storage_result) {
        g_task_think_storage_result = prototype_storage_load(&storage_data);
        g_task_think_storage_valid = CPU0_PROTOTYPE_STORAGE_OK == g_task_think_storage_result;
    }
    (void) task_infer_prototype_set(&storage_data, g_task_think_storage_valid);

    (void) stacd;
    (void) exinf;

    UW state_elapsed_ms = 0U;
    UW heartbeat_elapsed_ms = 0U;
    UW fault_elapsed_ms = 0U;
    UW last_observation_sequence = 0U;
    BOOL observation_sequence_valid = FALSE;
    UW last_infer_generation = 0U;
    UW target_sound_match_timer_ms = 0U;
    BOOL target_sound_matched = FALSE;

    while (1) {
        UINT fault_pattern = 0U;
        ER const flag_err =
            tk_wai_flg(think_fault_flag_id, CPU0_THINK_EVENT_MASK, TWF_ORW | TWF_BITCLR, &fault_pattern, TMO_POL);
        if (E_OK == flag_err) {
            UW const reported_faults = (UW) fault_pattern & APP_FAULT_ALL_MASK;
            g_task_think_fault_flags |= reported_faults;
            if ((0U != ((UW) fault_pattern & CPU0_THINK_EVENT_CLEAR_IPC_SEND)) &&
                (0U == (reported_faults & APP_FAULT_IPC_SEND))) {
                g_task_think_fault_flags &= ~((UW) APP_FAULT_IPC_SEND);
                fault_elapsed_ms = 0U;
            }
        }

        bsp_io_level_t sw1_level = BSP_IO_LEVEL_HIGH;
        (void) g_ioport.p_api->pinRead(g_ioport.p_ctrl, BSP_IO_PORT_00_PIN_09, &sw1_level);
        if (BSP_IO_LEVEL_LOW == sw1_level) {
            if (!learning_button_handled) {
                learning_button_press_ms += CPU0_THINK_PERIOD_MS;
                if (learning_button_press_ms >= 2000U) {
                    g_task_think_learning_mode = !g_task_think_learning_mode;
                    learning_button_handled = TRUE;
                    if (!g_task_think_learning_mode) {
                        if (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT == g_task_think_learning_samples) {
                            storage_candidate = storage_data;
                            UB const peak_bin = acoustic_identifier_find_peak_bin((const B *) storage_candidate.samples,
                                                                                  storage_candidate.sample_count);
                            storage_candidate.target_peak_bin = peak_bin;
                            /* 保存前の見本群から周波数binごとの重みを再計算する領域。 */
                            LOCAL float bin_weights[CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
                            acoustic_identifier_build_weights(peak_bin, bin_weights);
                            float threshold = 0.0F;
                            if (acoustic_identifier_leave_one_out_threshold((const B *) storage_candidate.samples,
                                                                            storage_candidate.sample_count,
                                                                            bin_weights, &threshold) &&
                                (E_OK == task_infer_background_export(&storage_candidate))) {
                                storage_candidate.identifier_threshold = threshold;
                                g_task_think_storage_result = prototype_storage_save(&storage_candidate);
                                if (CPU0_PROTOTYPE_STORAGE_OK == g_task_think_storage_result) {
                                    storage_data = storage_candidate;
                                    g_task_think_storage_valid = TRUE;
                                    (void) task_infer_prototype_set(&storage_data, TRUE);
                                }
                            }
                        }
                    } else {
                        g_task_think_learning_samples = 0U;
                        learning_last_feature_generation = g_task_infer_feature_generation;
                        storage_data.sample_count = 0U;
                        memset(storage_data.samples, 0, sizeof(storage_data.samples));
                    }
                }
            }
        } else {
            learning_button_press_ms = 0U;
            learning_button_handled = FALSE;
        }
        /* センサー取得結果を以降の安全判定と制御へ渡す再利用バッファ。 */
        LOCAL sensor_snapshot_t sensor_snapshot;
        memset(&sensor_snapshot, 0, sizeof(sensor_snapshot));
        ER const sensor_snapshot_err = task_sensor_snapshot_get(&sensor_snapshot);
        BOOL const sensor_fresh = task_think_sensor_fresh(&sensor_snapshot, sensor_snapshot_err);

        /* --- 音響リンクと推論 --- */
        /* USB音響リンクの最新状態を思考周期内で参照するスナップショット。 */
        LOCAL task_acoustic_link_snapshot_t snapshot;
        memset(&snapshot, 0, sizeof(snapshot));
        ER const snapshot_err = task_acoustic_link_snapshot_get(&snapshot);
        BOOL const observation_usable =
            (E_OK == snapshot_err) && snapshot.usb_configured && snapshot.hello_received &&
            snapshot.observation_received && (ACOUSTIC_XVF_STATUS_READY == snapshot.observation.xvf_status) &&
            (0U == (snapshot.observation.audio_flags &
                    (ACOUSTIC_AUDIO_FLAG_I2C_ERROR | ACOUSTIC_AUDIO_FLAG_MUTED | ACOUSTIC_AUDIO_FLAG_I2S_STALE)));
        BOOL const sequence_changed =
            observation_usable &&
            (!observation_sequence_valid || (snapshot.observation_sequence != last_observation_sequence));
        if (sequence_changed) {
            last_observation_sequence = snapshot.observation_sequence;
            g_task_think_observation_sequence = snapshot.observation_sequence;
            g_task_think_observation_watchdog_ms = 0U;
            observation_sequence_valid = TRUE;
        } else if (observation_sequence_valid &&
                   (g_task_think_observation_watchdog_ms <= UINT32_MAX - CPU0_THINK_PERIOD_MS)) {
            g_task_think_observation_watchdog_ms += CPU0_THINK_PERIOD_MS;
        }

        BOOL const link_ready = observation_usable &&
                                (snapshot.observation_age_ms <= CPU0_SOUND_OBSERVATION_TIMEOUT_MS) &&
                                (g_task_think_observation_watchdog_ms < CPU0_SOUND_OBSERVATION_TIMEOUT_MS);
        BOOL const new_observation = link_ready && sequence_changed;
        if (!observation_usable) {
            observation_sequence_valid = FALSE;
            g_task_think_observation_watchdog_ms = UINT32_MAX;
        }

        /* --- 音響識別タスクの最新結果取得とマッチング判定窓の更新 --- */
        /* 最新推論結果を音源一致判定へ渡す再利用バッファ。 */
        LOCAL task_infer_result_t infer_result;
        memset(&infer_result, 0, sizeof(infer_result));
        if (E_OK == task_infer_result_get(&infer_result)) {
            if (infer_result.feature_generation != last_infer_generation) {
                last_infer_generation = infer_result.feature_generation;
                if (CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_TARGET == infer_result.identifier.status) {
                    target_sound_matched = TRUE;
                    target_sound_match_timer_ms = CPU0_SOUND_IDENTIFIER_TIMEOUT_MS;
                } else if (CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_TARGET == infer_result.identifier.status) {
                    target_sound_matched = FALSE;
                    target_sound_match_timer_ms = 0U;
                }
            }
        }
        if (target_sound_match_timer_ms >= CPU0_THINK_PERIOD_MS) {
            target_sound_match_timer_ms -= CPU0_THINK_PERIOD_MS;
        } else {
            target_sound_match_timer_ms = 0U;
            target_sound_matched = FALSE;
        }

        /* --- S3: 音源追従コントローラを先に実行し、目標操舵角を取得 --- */
        BOOL const match_required = (0U != CPU0_SOUND_REQUIRE_IDENTIFIER_MATCH) && g_task_think_storage_valid;
        /* 音源追従コントローラへ渡す入力を思考周期ごとに更新する領域。 */
        LOCAL sound_follow_input_t input;
        input = (sound_follow_input_t){
            .link_ready = link_ready,
            .new_observation = new_observation,
            .fault_active = APP_FAULT_NONE != g_task_think_fault_flags,
            .motion_allowed = sensor_fresh && task_think_sound_motion_allowed(&sensor_snapshot),
            .observation = snapshot.observation,
            .match_required = match_required,
            .target_sound_matched = target_sound_matched,
        };
        /* 音源追従コントローラの出力を回避制御へ引き渡す領域。 */
        LOCAL sound_follow_output_t sf_output;
        memset(&sf_output, 0, sizeof(sf_output));
        sound_follow_state_t const previous_state = g_task_think_state;
        sound_follow_controller_step(&input, CPU0_THINK_PERIOD_MS, &sf_output);

        /* --- S3: 回避コントローラにtarget_steering_degとして音源方向を渡す --- */
        /* ToF回避コントローラの判断結果を安全調停へ渡す領域。 */
        LOCAL obstacle_avoidance_output_t oa_output;
        memset(&oa_output, 0, sizeof(oa_output));
        obstacle_avoidance_controller_step(sensor_fresh ? &sensor_snapshot : NULL,
                                           (APP_FAULT_NONE != g_task_think_fault_flags) || g_task_think_learning_mode,
                                           sensor_now_ms, sf_output.steering_deg, &oa_output);
        g_task_think_sensor_rule = oa_output.rule;

#if (CPU0_USE_CONTROL_MLP != 0U)
        LOCAL control_mlp_output_t mlp_output;
        memset(&mlp_output, 0, sizeof(mlp_output));
        control_mlp_planner_step(sensor_fresh ? &sensor_snapshot : NULL,
                                 (float) sf_output.steering_deg,
                                 &mlp_output);
#endif

        /* --- S4: 連続走行マージ（音源引力とToF斥力を合成した回避走行指令を採用） --- */
        /* 追従・回避・停止の優先順位を反映した最終要求値。 */
        LOCAL sound_follow_output_t output;
        memset(&output, 0, sizeof(output));
        BOOL const tracking_active = (CPU0_THINK_STATE_MOVE_STEP == sf_output.state);
        BOOL const avoidance_active = (oa_output.rule == CPU0_SENSOR_RULE_BACKUP) ||
                                      (oa_output.rule == CPU0_SENSOR_RULE_PIVOT_LEFT) ||
                                      (oa_output.rule == CPU0_SENSOR_RULE_PIVOT_RIGHT);

#if (CPU0_USE_CONTROL_MLP != 0U)
        if (!mlp_output.fallback_required && !avoidance_active && !oa_output.emergency_stop && !sf_output.emergency_stop &&
            !g_task_think_learning_mode && (APP_FAULT_NONE == g_task_think_fault_flags)) {
            /* TFLM 制御MLP による連続回避・追従計画 */
            output.state = sf_output.state;
            output.steering_deg = (H) roundf(mlp_output.steering_deg);
            output.emergency_stop = FALSE;

            if (mlp_output.is_blocked || (mlp_output.speed_scale <= 0.01f)) {
                output.left_rpm = 0;
                output.right_rpm = 0;
                output.actuator_enable = FALSE;
            } else if (tracking_active) {
                /* 始動トルク下限（85 RPM）を確保し、モーター不感帯・静止摩擦による停止を防止 */
                W const rpm_range = (W) CPU0_SENSOR_FORWARD_RPM - CPU0_SENSOR_MIN_FORWARD_RPM;
                H const base_rpm = (H) (CPU0_SENSOR_MIN_FORWARD_RPM +
                                        (H) roundf(mlp_output.speed_scale * (float) rpm_range));

                /* 旋回舵角に応じた左右差動配分（外輪増速・内輪下限ガード） */
                W const steer_mag = (mlp_output.steering_deg < 0.0f) ?
                                    (W) (-mlp_output.steering_deg) : (W) (mlp_output.steering_deg);
                W const inner_slowdown = (steer_mag * 25) / CPU0_SENSOR_STEERING_MAX_DEG;
                H inner_rpm = (H) (((W) base_rpm * (100 - inner_slowdown)) / 100);
                if (inner_rpm < CPU0_SENSOR_MIN_FORWARD_RPM) {
                    inner_rpm = (H) CPU0_SENSOR_MIN_FORWARD_RPM;
                }
                W const outer_boost = (steer_mag * 10) / CPU0_SENSOR_STEERING_MAX_DEG;
                H outer_rpm = (H) (((W) base_rpm * (100 + outer_boost)) / 100);
                if (outer_rpm > 130) {
                    outer_rpm = 130;
                }

                if (mlp_output.steering_deg < 0.0f) {
                    /* 左旋回: 左が内輪、右が外輪 */
                    output.left_rpm = inner_rpm;
                    output.right_rpm = outer_rpm;
                } else if (mlp_output.steering_deg > 0.0f) {
                    /* 右旋回: 右が内輪、左が外輪 */
                    output.left_rpm = outer_rpm;
                    output.right_rpm = inner_rpm;
                } else {
                    /* 直進 */
                    output.left_rpm = base_rpm;
                    output.right_rpm = base_rpm;
                }
                output.actuator_enable = TRUE;
            } else if (CPU0_THINK_STATE_STEER_PREP == sf_output.state) {
                output.left_rpm = 0;
                output.right_rpm = 0;
                output.actuator_enable = TRUE;
            } else {
                output.left_rpm = 0;
                output.right_rpm = 0;
                output.actuator_enable = FALSE;
            }
        } else
#endif
        if (oa_output.emergency_stop || sf_output.emergency_stop) {
            /* 緊急停止要求 */
            output = sf_output;
            output.emergency_stop  = TRUE;
            output.actuator_enable = FALSE;
            output.left_rpm        = 0;
            output.right_rpm       = 0;
        } else if (avoidance_active && oa_output.actuator_enable) {
            /* 障害物回避の切り返し・脱出動作を最優先実行 */
            output.state           = oa_output.state;
            output.steering_deg    = oa_output.steering_deg;
            output.left_rpm        = oa_output.left_rpm;
            output.right_rpm       = oa_output.right_rpm;
            output.actuator_enable = TRUE;
            output.emergency_stop  = FALSE;
        } else if (tracking_active && oa_output.actuator_enable) {
            /* S4連続走行: 音源引力とToF斥力を合成した回避走行指令を採用 */
            output.state           = oa_output.state;
            output.steering_deg    = oa_output.steering_deg;
            output.left_rpm        = oa_output.left_rpm;
            output.right_rpm       = oa_output.right_rpm;
            output.actuator_enable = TRUE;
            output.emergency_stop  = FALSE;
        } else if (CPU0_THINK_STATE_STEER_PREP == sf_output.state) {
            /* 初回音源検知時の操舵整定（前進せず舵角のみ準備） */
            output = sf_output;
            output.left_rpm        = 0;
            output.right_rpm       = 0;
            output.actuator_enable = TRUE;
        } else {
            /* 待機・静定・安全停止: 走行停止で音源を待つ */
            output = sf_output;
            output.left_rpm        = 0;
            output.right_rpm       = 0;
        }

        g_task_think_state = output.state;
        if (previous_state != g_task_think_state) {
            state_elapsed_ms = 0U;
        }

        if (g_task_think_learning_mode) {
            output.emergency_stop = TRUE;
            output.actuator_enable = FALSE;
            output.steering_deg = 0;
            output.left_rpm = 0;
            output.right_rpm = 0;
            task_think_learning_capture(&snapshot, observation_usable);
        }

        /* --- S3: safety_arbiter 経由で安全クランプしてから発行 --- */
        /* 安全調停前の要求値を明示的に分離して監視可能にする領域。 */
        LOCAL safety_motion_command_t sa_requested;
        sa_requested = (safety_motion_command_t){
            .steering_deg   = output.steering_deg,
            .left_rpm       = output.left_rpm,
            .right_rpm      = output.right_rpm,
            .actuator_enable = output.actuator_enable,
            .emergency_stop = output.emergency_stop,
        };
        /* ToF・生存性・IMUの安全条件を適用した出力値。 */
        LOCAL safety_motion_command_t sa_arbitrated;
        memset(&sa_arbitrated, 0, sizeof(sa_arbitrated));
        safety_arbiter_arbitrate(&sa_requested, &sensor_snapshot, sensor_fresh, &sa_arbitrated);

        /* 調停後の値を出力に反映 */
        output.steering_deg   = sa_arbitrated.steering_deg;
        output.left_rpm       = sa_arbitrated.left_rpm;
        output.right_rpm      = sa_arbitrated.right_rpm;
        output.actuator_enable = sa_arbitrated.actuator_enable;
        output.emergency_stop = sa_arbitrated.emergency_stop;

        if (E_OK != task_think_publish_target(&output)) {
            g_task_think_fault_flags |= APP_FAULT_TARGET_UPDATE;
            input.fault_active = TRUE;
            sound_follow_controller_step(&input, 0U, &sf_output);
            output = sf_output;
            output.emergency_stop = TRUE;
            output.actuator_enable = FALSE;
            output.left_rpm = 0;
            output.right_rpm = 0;
            g_task_think_state = output.state;
            (void) task_think_publish_target(&output);
        }

        g_task_think_link_ready = link_ready;
        g_task_think_new_observation = new_observation;
        g_task_think_steering_deg = output.steering_deg;
        g_task_think_left_rpm = output.left_rpm;
        g_task_think_right_rpm = output.right_rpm;
        g_task_think_actuator_enable = output.actuator_enable;
        g_task_think_emergency_stop = output.emergency_stop;

        task_think_led_update(state_elapsed_ms, heartbeat_elapsed_ms, fault_elapsed_ms);
        g_task_think_cycle_count++;
        state_elapsed_ms += CPU0_THINK_PERIOD_MS;

        if (APP_FAULT_NONE == g_task_think_fault_flags) {
            heartbeat_elapsed_ms += CPU0_THINK_PERIOD_MS;
            if (heartbeat_elapsed_ms >= CPU0_LED_HEARTBEAT_PERIOD_MS) {
                heartbeat_elapsed_ms = 0U;
            }
        } else {
            fault_elapsed_ms += CPU0_THINK_PERIOD_MS;
        }

        (void) tk_dly_tsk(CPU0_THINK_PERIOD_MS);
    }
}

/** =================================================================*
 * @brief  タスク起動不能時の2LED異常表示
 * @param[in] fault 表示するCPU0異常
 * ================================================================= */
EXPORT void task_think_halt(app_fault_t fault) {
    UW fault_elapsed_ms = 0U;
    g_task_think_fault_flags |= (UW) fault;
    g_task_think_state = CPU0_THINK_STATE_FAULT;
    g_task_think_link_ready = FALSE;
    g_task_think_new_observation = FALSE;
    g_task_think_steering_deg = 0;
    g_task_think_left_rpm = 0;
    g_task_think_right_rpm = 0;
    g_task_think_actuator_enable = FALSE;
    g_task_think_emergency_stop = TRUE;

    while (1) {
        task_think_led_update(0U, 0U, fault_elapsed_ms);
        fault_elapsed_ms += CPU0_THINK_PERIOD_MS;
        (void) tk_dly_tsk(CPU0_THINK_PERIOD_MS);
    }
}
