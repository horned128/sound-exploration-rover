/** =================================================================*
 * @file   task_think.c
 * @brief  CPU0自律走行・現場学習タスク
 * ================================================================= */
#include "task_think.h"                                     /* CPU0思考タスクAPI */
#include "config/control_config.h"                          /* 思考モードと走行値 */
#include "config/pin_config.h"                              /* LEDの役割設定 */
#include "config/sensor_config.h"                           /* センサー安全判定値 */
#include "config/task_config.h"                             /* 思考周期と優先度 */
#include "ipc/actuator_ipc_client.h"                        /* CPU1実測左右輪速度 */
#include "control/control_mlp_planner.h"                    /* TFLM制御MLPプランナ */
#include "control/debug_motor_recording.h"                  /* SW短押しの期限付き録音走行 */
#include "control/obstacle_avoidance_controller.h"          /* ToF・IMU走行判断 */
#include "control/safety_arbiter.h"                         /* 安全調停・ToF veto集約 */
#include "control/sensor_liveness.h"                        /* 取得タスクと独立した更新監視 */
#include "control/sound_follow_controller.h"                /* 音源追従状態機械 */
#include <math.h>                                           /* roundf */
#include "hal_data.h"                                       /* BSP LED情報、ピンAPI */
#include "services/acoustic_identifier.h"                   /* 見本leave-one-outしきい値算出 */
#include "services/prototype_storage.h"                     /* Code MRAMプロトタイプ保存 */
#include "services/odometry.h"                              /* 車輪・IMUオドメトリ */
#include "services/sound_source_localizer.h"                /* bearing-only音源位置推定 */
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
LOCAL ER task_think_publish_motion(H steering_deg, BOOL is_spin_turn, H left_rpm, H right_rpm,
                                   BOOL actuator_enable, BOOL emergency_stop); /* 4輪目標展開 */
/* 音源追従の近接安全判定 */
LOCAL BOOL task_think_sound_motion_allowed(const sensor_snapshot_t * p_snapshot); /* 音源追従走行可否判定 */
LOCAL H task_think_spin_breakaway_rpm_update(BOOL is_spin_turn, BOOL moving, H gyro_z_dps_x10); /* スピンターン始動探索 */
LOCAL void task_think_led_write(BOOL blue_on, BOOL green_on); /* 2LED一括更新 */
LOCAL void task_think_learning_capture(const task_acoustic_link_snapshot_t * p_snapshot,
                                       BOOL observation_usable); /* 新規特徴量パッチの学習 */
LOCAL void task_think_learning_start(void);                  /* 新しい見本収集を初期化 */
LOCAL void task_think_learning_commit(void);                 /* 5見本をMRAMへ保存 */
LOCAL void task_think_learning_cancel(void);                 /* 未保存見本を破棄 */
LOCAL void task_think_learning_command_apply(void);          /* キュー済み学習操作を反映 */

#if (CPU0_DEBUG_MOTOR_RECORDING_ENABLE == 0U)
LOCAL UW learning_button_press_ms;                          /**< SW1継続押下時間[ms] */
LOCAL BOOL learning_button_handled;                         /**< 同一押下の多重切替防止 */
#endif
LOCAL debug_motor_recording_t debug_motor_state;            /**< 収録用デバッグ走行とSW状態 */
LOCAL UW learning_last_feature_generation;                  /**< 最後に収集した特徴量世代 */
LOCAL volatile task_think_learning_command_t learning_command_pending; /**< 次周期に反映する外部操作 */
LOCAL volatile BOOL restart_request_pending;                /**< 次周期に反映する再開操作 */
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
EXPORT volatile UH g_task_think_raw_doa_deg;                /**< XVF3800 raw DoA[deg] */
EXPORT volatile UH g_task_think_filtered_doa_deg;           /**< ESP32S3循環平均DoA[deg] */
EXPORT volatile UB g_task_think_doa_confidence;             /**< DoA品質[0..100] */
EXPORT volatile W g_task_think_rover_x_mm;                  /**< 推定車体X座標[mm] */
EXPORT volatile W g_task_think_rover_y_mm;                  /**< 推定車体Y座標[mm] */
EXPORT volatile W g_task_think_rover_heading_mrad;          /**< 推定車体方位[mrad] */
EXPORT volatile W g_task_think_source_x_mm;                 /**< 推定音源X座標[mm] */
EXPORT volatile W g_task_think_source_y_mm;                 /**< 推定音源Y座標[mm] */
EXPORT volatile UW g_task_think_source_range_mm;            /**< 推定音源距離[mm] */
EXPORT volatile H g_task_think_source_bearing_deg;          /**< 音源目標方位（右正）[deg] */
EXPORT volatile UB g_task_think_source_confidence;          /**< 音源位置品質[0..100] */
EXPORT volatile UB g_task_think_localization_observation_count; /**< 位置推定観測数 */
EXPORT volatile UH g_task_think_localization_residual_mm;   /**< 方位線残差RMS[mm] */
EXPORT volatile UH g_task_think_localization_crossing_deg;  /**< 方位交差角[deg] */
EXPORT volatile UH g_task_think_localization_baseline_mm;   /**< 方位観測の最大基線[mm] */
EXPORT volatile UH g_task_think_source_position_shift_mm;   /**< 前回推定からの位置変化[mm] */
EXPORT volatile BOOL g_task_think_localization_geometry_valid; /**< 今回の推定幾何有効 */
EXPORT volatile BOOL g_task_think_source_position_valid;    /**< 音源位置推定有効 */
EXPORT volatile BOOL g_task_think_navigation_target_valid;  /**< 音源目標保持期限内 */
EXPORT volatile BOOL g_task_think_arrival_candidate;        /**< 音源到着候補 */
EXPORT volatile sound_arrival_state_t g_task_think_arrival_state; /**< 到着判定段階 */
EXPORT volatile UB g_task_think_arrival_confirm_count;      /**< 到着確認観測数 */
EXPORT volatile UW g_task_think_autonomous_backup_count;    /**< 自律両輪後退検出数 */
/**< 選択センサー走行ルール */
EXPORT volatile obstacle_avoidance_rule_t g_task_think_sensor_rule;
EXPORT volatile UW g_task_think_fault_flags;                /**< CPU0異常ラッチ */
EXPORT volatile BOOL g_task_think_learning_mode;            /**< 現場学習モード */
EXPORT volatile BOOL g_task_think_debug_motor_active;       /**< PCM収録用デバッグ走行中 */
EXPORT volatile UW g_task_think_debug_motor_remaining_ms;   /**< 自動停止まで[ms] */
EXPORT volatile UB g_task_think_learning_samples;           /**< 収集済み音響見本数 */
EXPORT volatile BOOL g_task_think_storage_valid;            /**< 有効なMRAMプロトタイプ有無 */
/**< 直近MRAM処理結果 */
EXPORT volatile prototype_storage_result_t g_task_think_storage_result;

/** =================================================================*
 * @brief  音源追従で前進してよいToF状態か判定
 * @details センサー取得失敗、更新期限超過、またはToF無効なら走行を許可しない。
 *          距離閾値による即時停止と正面衝突候補の連続確認は回避制御側で扱う。
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 3台のToFが有効かつ期限内ならtrue
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
    g_task_think_raw_doa_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;
    g_task_think_filtered_doa_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;
    g_task_think_doa_confidence = 0U;
    g_task_think_rover_x_mm = 0;
    g_task_think_rover_y_mm = 0;
    g_task_think_rover_heading_mrad = 0;
    g_task_think_source_x_mm = 0;
    g_task_think_source_y_mm = 0;
    g_task_think_source_range_mm = 0U;
    g_task_think_source_bearing_deg = 0;
    g_task_think_source_confidence = 0U;
    g_task_think_localization_observation_count = 0U;
    g_task_think_localization_residual_mm = 0U;
    g_task_think_localization_crossing_deg = 0U;
    g_task_think_localization_baseline_mm = 0U;
    g_task_think_source_position_shift_mm = 0U;
    g_task_think_localization_geometry_valid = FALSE;
    g_task_think_source_position_valid = FALSE;
    g_task_think_navigation_target_valid = FALSE;
    g_task_think_arrival_candidate = FALSE;
    g_task_think_arrival_state = CPU0_SOUND_ARRIVAL_SEARCH;
    g_task_think_arrival_confirm_count = 0U;
    g_task_think_autonomous_backup_count = 0U;
    g_task_think_sensor_rule = CPU0_SENSOR_RULE_SAFE_STOP;
    g_task_think_fault_flags = APP_FAULT_NONE;
    g_task_think_learning_mode = FALSE;
    g_task_think_debug_motor_active = FALSE;
    g_task_think_debug_motor_remaining_ms = 0U;
    memset(&debug_motor_state, 0, sizeof(debug_motor_state));
    g_task_think_learning_samples = 0U;
    g_task_think_storage_valid = FALSE;
    g_task_think_storage_result = CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
    learning_command_pending = TASK_THINK_LEARNING_COMMAND_NONE;
    restart_request_pending = FALSE;
    sound_follow_controller_init();
    sound_source_localizer_init();
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
 * @brief 現場学習の開始・保存・取消を思考タスクへ依頼
 * @details USB受信taskはMRAMや見本バッファを直接触らず、思考周期の先頭で
 *          処理させる。これによりSW1とPC操作の競合を避ける。
 * @param[in] command 現場学習操作
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER task_think_learning_request(task_think_learning_command_t command) {
    if ((TASK_THINK_LEARNING_COMMAND_START != command) &&
        (TASK_THINK_LEARNING_COMMAND_COMMIT != command) &&
        (TASK_THINK_LEARNING_COMMAND_CANCEL != command)) {
        return E_PAR;
    }
    if (!think_task_started) {
        return E_NOEXS;
    }
    learning_command_pending = command;
    return E_OK;
}

/** =================================================================*
 * @brief 停止状態からの明示再開を思考タスクへ依頼
 * @details USB受信taskから直接走行状態を変更せず、次の思考周期でだけ反映する。
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER task_think_restart_request(void) {
    if (!think_task_started) {
        return E_NOEXS;
    }
    restart_request_pending = TRUE;
    return E_OK;
}

/** =================================================================*
 * @brief 旧見本をRAM/MRAMから消して新しい現場見本の収集を開始
 * ================================================================= */
LOCAL void task_think_learning_start(void) {
    if (g_task_think_learning_mode) {
        return;
    }
    g_task_think_learning_mode = TRUE;
    g_task_think_learning_samples = 0U;
    learning_last_feature_generation = g_task_infer_feature_generation;
    g_task_think_storage_valid = FALSE;
    memset(&storage_data, 0, sizeof(storage_data));
    /* MRAM消去中も旧見本を推論に使わせない。 */
    if (E_OK != task_infer_prototype_set(&storage_data, FALSE)) {
        g_task_think_storage_result = CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
        g_task_think_learning_mode = FALSE;
        return;
    }
    /* MRAM書込み中はCPU1も停止する。走行中のSW1操作に備え、先に
     * emergency stopを指令し、優先度の高い指令タスクが送る時間を確保する。 */
    if (E_OK != task_think_publish_motion(0, FALSE, 0, 0, FALSE, TRUE)) {
        g_task_think_fault_flags |= APP_FAULT_TARGET_UPDATE;
        g_task_think_storage_result = CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
        g_task_think_learning_mode = FALSE;
        return;
    }
    if (E_OK != tk_dly_tsk(CPU0_COMMAND_PERIOD_MS * 2U)) {
        g_task_think_storage_result = CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
        g_task_think_learning_mode = FALSE;
        return;
    }
    g_task_think_storage_result = prototype_storage_clear();
    if (CPU0_PROTOTYPE_STORAGE_OK != g_task_think_storage_result) {
        g_task_think_learning_mode = FALSE;
    } else {
        /* 初期化待ちの間に完成した特徴量は新しい学習見本へ混ぜない。 */
        learning_last_feature_generation = g_task_infer_feature_generation;
    }
}

/** =================================================================*
 * @brief 完成した5見本を検証してMRAMへ保存
 * @details 見本が不足した保存要求は収集状態を維持する。
 * ================================================================= */
LOCAL void task_think_learning_commit(void) {
    if (!g_task_think_learning_mode) {
        return;
    }
    if (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT != g_task_think_learning_samples) {
        g_task_think_storage_result = CPU0_PROTOTYPE_STORAGE_EMPTY;
        return;
    }

    storage_candidate = storage_data;
    UB peak_bin = acoustic_identifier_find_peak_bin((const B *) storage_candidate.samples,
                                                          storage_candidate.sample_count);
    storage_candidate.target_peak_bin = peak_bin;
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
    } else {
        g_task_think_storage_result = CPU0_PROTOTYPE_STORAGE_EMPTY;
    }
    g_task_think_learning_mode = FALSE;
}

/** =================================================================*
 * @brief 未保存の現場見本を破棄
 * ================================================================= */
LOCAL void task_think_learning_cancel(void) {
    g_task_think_learning_mode = FALSE;
    g_task_think_learning_samples = 0U;
    storage_data.sample_count = 0U;
    memset(storage_data.samples, 0, sizeof(storage_data.samples));
}

/** =================================================================*
 * @brief 保留中のPC/SW1学習操作を思考タスク文脈で反映
 * ================================================================= */
LOCAL void task_think_learning_command_apply(void) {
    task_think_learning_command_t command = learning_command_pending;
    learning_command_pending = TASK_THINK_LEARNING_COMMAND_NONE;
    if (TASK_THINK_LEARNING_COMMAND_START == command) {
        task_think_learning_start();
    } else if (TASK_THINK_LEARNING_COMMAND_COMMIT == command) {
        task_think_learning_commit();
    } else if (TASK_THINK_LEARNING_COMMAND_CANCEL == command) {
        task_think_learning_cancel();
    }
}

/** =================================================================*
 * @brief  最新の192次元要約を1回だけ現場見本として取り込む
 * @details 能動フレーム不足は判定不能なので、零ベクトルを見本として保存しない。
 * @param[in] p_snapshot 最新の音響リンク状態
 * @param[in] observation_usable 音響観測の有効状態
 * ================================================================= */
LOCAL void task_think_learning_capture(const task_acoustic_link_snapshot_t * p_snapshot,
                                       BOOL observation_usable) {
    if (!g_task_think_learning_mode ||
        (g_task_think_learning_samples >= CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT)) {
        return;
    }
    if ((NULL == p_snapshot) || (!observation_usable)) {
        return;
    }

    /* 特徴量パッチはESP32S3側で音量トリガ済み。
     * 推論完了時点のVAD/音量は収録時点と異なるため、ここでは再判定しない。 */

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
    if (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT == g_task_think_learning_samples) {
        UB const mask = acoustic_identifier_consensus_mask((const B *) storage_data.samples,
                                                            g_task_think_learning_samples);
        if (mask != ((1U << CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT) - 1U)) {
            /* 4/5または3/5が一意に合意した場合のみ、異音の1〜2件を再取得する。 */
            UW kept = 0U;
            for (UW index = 0U; index < CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT; index++) {
                if (mask & (1U << index)) {
                    if (kept != index) {
                        memcpy(storage_data.samples[kept], storage_data.samples[index],
                               sizeof(storage_data.samples[0]));
                    }
                    kept++;
                }
            }
            memset(&storage_data.samples[kept], 0,
                   (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT - kept) * sizeof(storage_data.samples[0]));
            g_task_think_learning_samples = (UB) kept;
        }
    }
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
 * @details 通常時は前後輪を逆相操舵し、最小並進回頭時はX字へ展開する。
 * @param[in] p_output 音源追従状態機械の出力
 * @return μT-Kernelエラーコード
 * ================================================================= */
LOCAL ER task_think_publish_target(const sound_follow_output_t * p_output) {
    if (NULL == p_output) {
        return E_PAR;
    }

    return task_think_publish_motion(p_output->steering_deg, p_output->is_spin_turn, p_output->left_rpm,
                                     p_output->right_rpm, p_output->actuator_enable, p_output->emergency_stop);
}

LOCAL H s_spin_current_rpm = CPU0_SPIN_RAMP_START_RPM;
LOCAL BOOL s_spin_breakaway_detected = FALSE;

/** =================================================================*
 * @brief  始動トルク探索型スピンターンRPM決定（Breakaway & Hold）
 * @details 6輪ロッカーボギー機構の対角突っ張り・車輪浮き・空転・ストールを防ぐため、
 *          低RPMから徐々にランプアップし、ジャイロによる回転開始検知時のRPMをホールドする。
 * @param[in] is_spin_turn スピンターン状態
 * @param[in] moving 回転要求あり
 * @param[in] gyro_z_dps_x10 最新鉛直ジャイロ角速度[0.1dps]
 * @return 適用するスピンターン目標RPM絶対値
 * ================================================================= */
LOCAL H task_think_spin_breakaway_rpm_update(BOOL is_spin_turn, BOOL moving, H gyro_z_dps_x10) {
    if (!is_spin_turn || !moving) {
        s_spin_current_rpm = CPU0_SPIN_RAMP_START_RPM;
        s_spin_breakaway_detected = FALSE;
        return 0;
    }

    H const abs_gyro = (gyro_z_dps_x10 < 0) ? (H) -gyro_z_dps_x10 : gyro_z_dps_x10;

    /* ジャイロが変化している（実際に回転している: >= 6.0 dps）か判定 */
    if (abs_gyro >= CPU0_SPIN_MOTION_DETECT_DPS_X10) {
        /* 回転中: これ以上のランプアップを止め、現在のRPMをホールド */
        s_spin_breakaway_detected = TRUE;

        /* 回転が速すぎる場合（過剰トルク・浮き上がり防止）はマイルドに下げる */
        if (abs_gyro > CPU0_SPIN_HOLD_MAX_DPS_X10) {
            if (s_spin_current_rpm - 3 >= CPU0_SPIN_RAMP_START_RPM) {
                s_spin_current_rpm = (H) (s_spin_current_rpm - 3);
            }
        }
    } else {
        /* ジャイロが変化していない（回っていない、または停止した）:
         * ジャイロが変化するまで毎周期Dutyを上げ続ける！ */
        s_spin_breakaway_detected = FALSE;
        if (s_spin_current_rpm + CPU0_SPIN_RAMP_STEP_RPM <= CPU0_SPIN_RAMP_MAX_RPM) {
            s_spin_current_rpm = (H) (s_spin_current_rpm + CPU0_SPIN_RAMP_STEP_RPM);
        } else {
            s_spin_current_rpm = CPU0_SPIN_RAMP_MAX_RPM;
        }
    }

    return s_spin_current_rpm;
}

/** =================================================================*
 * @brief  共通走行指令を4輪操舵・左右DCモーターの目標へ展開
 * @details 通常時は前後輪逆相、最小並進回頭時はX字操舵で左右DCモーターを指令する。
 * @param[in] steering_deg 右正の車体操舵角
 * @param[in] is_spin_turn 最小並進回頭のX字操舵を選択する状態
 * @param[in] left_rpm 論理左モーター目標RPM
 * @param[in] right_rpm 論理右モーター目標RPM
 * @param[in] actuator_enable 出力許可
 * @param[in] emergency_stop 非常停止指定
 * @return μT-Kernelエラーコード
 * ================================================================= */
LOCAL ER task_think_publish_motion(H steering_deg, BOOL is_spin_turn, H left_rpm, H right_rpm,
                                   BOOL actuator_enable, BOOL emergency_stop) {

    rover_motion_target_t target = {
        .left_target_rpm = left_rpm,
        .right_target_rpm = right_rpm,
        .actuator_enable = actuator_enable,
        .emergency_stop = emergency_stop,
    };

    if (is_spin_turn) {
        /*
         * 6輪ロッカーボギーサスペンションのX字操舵による超信地旋回。
         * 前後4輪をX字（35°）に配向し、探索・調停済みRPMで左右逆回転を行う。
         */
        H const spin_servo_deg = (H) (CPU0_STEERING_SERVO_OUTPUT_SIGN * CPU0_SOUND_SPIN_SERVO_DEG);
        target.servo_target_deg[0] = spin_servo_deg;
        target.servo_target_deg[1] = (H) -spin_servo_deg;
        target.servo_target_deg[2] = (H) -spin_servo_deg;
        target.servo_target_deg[3] = spin_servo_deg;
    } else {
        H const front_steering_deg = (H) (CPU0_STEERING_SERVO_OUTPUT_SIGN * steering_deg);
        target.servo_target_deg[0] = front_steering_deg;
        target.servo_target_deg[1] = front_steering_deg;
        target.servo_target_deg[2] = (H) -front_steering_deg;
        target.servo_target_deg[3] = (H) -front_steering_deg;
    }

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

        blue_on = (position_ms < pulse_window_ms) && (0U == ((position_ms / CPU0_LED_FAULT_PULSE_MS) & 1U));
    } else {
        switch (g_task_think_state) {
        case CPU0_THINK_STATE_WAIT_LINK:
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_WAIT_LINK_BLINK_MS) & 1U);
            break;

        case CPU0_THINK_STATE_LISTEN:
            blue_on = state_elapsed_ms % CPU0_LED_LISTEN_BLINK_MS < CPU0_LED_HEARTBEAT_PULSE_MS;
            break;

        case CPU0_THINK_STATE_STEER_PREP:
        case CPU0_THINK_STATE_SPIN_PREP:
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_STEER_BLINK_MS) & 1U);
            break;

        case CPU0_THINK_STATE_MOVE_STEP:
        case CPU0_THINK_STATE_SPIN_STEP:
            blue_on = TRUE;
            break;

        case CPU0_THINK_STATE_SETTLE:
        case CPU0_THINK_STATE_COOLDOWN:
            blue_on = 0U == ((state_elapsed_ms / CPU0_LED_SETTLE_BLINK_MS) & 1U);
            break;

        case CPU0_THINK_STATE_WAIT_RESTART:
            /* 終了停止中の青LEDは消し、緑LEDは学習状態を示し続ける。 */
            blue_on = FALSE;
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

    /* 緑LEDは審査員の学習スイッチ専用: 収集中はゆっくり点滅、5件で速く点滅、
     * 保存済みは点灯、MRAM異常は2回短点滅、未学習は消灯。 */
    if (g_task_think_learning_mode) {
        green_on = (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT == g_task_think_learning_samples) ?
            (0U == ((heartbeat_elapsed_ms / 100U) & 1U)) : (heartbeat_elapsed_ms < 500U);
    } else if (g_task_think_storage_valid &&
               (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT == storage_data.sample_count)) {
        green_on = TRUE;
    } else if ((CPU0_PROTOTYPE_STORAGE_OK != g_task_think_storage_result) &&
               (CPU0_PROTOTYPE_STORAGE_EMPTY != g_task_think_storage_result) &&
               (CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED != g_task_think_storage_result)) {
        green_on = (heartbeat_elapsed_ms < 100U) ||
                   ((heartbeat_elapsed_ms >= 200U) && (heartbeat_elapsed_ms < 300U));
    }
    task_think_led_write(blue_on, green_on);
}

/** =================================================================*
 * @brief  思考タスク本体
 * @param[in] stacd 起動コード（本関数では未使用）
 * @param[in] exinf 起動情報（本関数では未使用）
 * ================================================================= */
LOCAL void task_think_entry(INT stacd, void * exinf) {
    g_task_think_storage_result = prototype_storage_init();
    if (CPU0_PROTOTYPE_STORAGE_OK == g_task_think_storage_result) {
        g_task_think_storage_result = prototype_storage_load(&storage_data);
        g_task_think_storage_valid = CPU0_PROTOTYPE_STORAGE_OK == g_task_think_storage_result;
    }
    if (g_task_think_storage_valid && (storage_data.sample_count > 0U)) {
        storage_data.target_peak_bin = acoustic_identifier_consensus_peak_bin(
            (const B *) storage_data.samples, storage_data.sample_count);
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
    BOOL target_sound_direction_valid = FALSE;
    BOOL avoidance_motion_latched = FALSE;

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

        /* PC直結AIラボの要求は、SW1判定より先に同じ思考文脈で適用する。 */
        task_think_learning_command_apply();
        BOOL const restart_request = restart_request_pending;
        restart_request_pending = FALSE;

        bsp_io_level_t sw1_level = BSP_IO_LEVEL_HIGH;
        (void) g_ioport.p_api->pinRead(g_ioport.p_ctrl, BSP_IO_PORT_00_PIN_09, &sw1_level);
#if (CPU0_DEBUG_MOTOR_RECORDING_ENABLE != 0U)
        debug_motor_button_t const sw_event = debug_motor_button_step(&debug_motor_state,
            BSP_IO_LEVEL_LOW == sw1_level, CPU0_THINK_PERIOD_MS);
        BOOL learning_long_press = FALSE;
        if (DEBUG_MOTOR_BUTTON_LONG == sw_event) {
            if (debug_motor_state.active || debug_motor_state.press_started_during_drive) {
                /* デバッグ走行中の長押しは停止だけ。MRAM学習を開始しない。 */
                debug_motor_recording_step(&debug_motor_state, sw_event, FALSE, CPU0_THINK_PERIOD_MS);
            } else {
                learning_long_press = TRUE;
            }
        }
#else
        BOOL learning_long_press = FALSE;
        if (BSP_IO_LEVEL_LOW == sw1_level) {
            if (!learning_button_handled) {
                learning_button_press_ms += CPU0_THINK_PERIOD_MS;
                if (learning_button_press_ms >= 2000U) {
                    learning_button_handled = TRUE;
                    learning_long_press = TRUE;
                }
            }
        } else {
            learning_button_press_ms = 0U;
            learning_button_handled = FALSE;
        }
#endif
        if (learning_long_press) {
            if (!g_task_think_learning_mode) {
                task_think_learning_start();
            } else if (CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT == g_task_think_learning_samples) {
                task_think_learning_commit();
            } else {
                task_think_learning_cancel();
            }
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
            (snapshot.observation.doa_deg < 360U) && (snapshot.observation.raw_doa_deg < 360U) &&
            (snapshot.observation.doa_confidence <= 100U) &&
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
                    /* ピーク帯域と距離の判定を通過したTARGETは、直ちにDoA取得へ渡す。 */
                    target_sound_matched = TRUE;
                    target_sound_direction_valid = TRUE;
                    target_sound_match_timer_ms = CPU0_SOUND_IDENTIFIER_TIMEOUT_MS;
                } else {
                    /* 未一致パッチのDoAで方向を更新しない。ピーク帯域が見本と一致する
                     * NOT_TARGETは走行雑音で距離が揺れ得るため、一致保持を短く継続する。 */
                    target_sound_direction_valid = FALSE;
                    UB const current_peak_bin = infer_result.summary_valid ?
                        acoustic_identifier_find_peak_bin(infer_result.summary, 1U) : 255U;
                    W const peak_delta = (W) current_peak_bin - (W) storage_data.target_peak_bin;
                    BOOL const peak_matches = (current_peak_bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT) &&
                        (storage_data.target_peak_bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT) &&
                        (peak_delta <= (W) CPU0_ACOUSTIC_IDENTIFIER_PEAK_TOLERANCE_BINS) &&
                        (peak_delta >= -(W) CPU0_ACOUSTIC_IDENTIFIER_PEAK_TOLERANCE_BINS);
                    float const distance = infer_result.identifier.minimum_cosine_distance;
                    if ((CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_TARGET == infer_result.identifier.status) &&
                        ((distance >= CPU0_SOUND_IDENTIFIER_FORCE_REJECT_DISTANCE) ||
                         ((distance >= CPU0_SOUND_IDENTIFIER_REJECT_DISTANCE) && !peak_matches))) {
                        target_sound_matched = FALSE;
                        target_sound_match_timer_ms = 0U;
                    }
                }
            }
        }
        if (0U == target_sound_match_timer_ms) {
            target_sound_matched = FALSE;
            target_sound_direction_valid = FALSE;
        } else if (target_sound_match_timer_ms >= CPU0_THINK_PERIOD_MS) {
            target_sound_match_timer_ms -= CPU0_THINK_PERIOD_MS;
        } else {
            target_sound_match_timer_ms = 0U;
            target_sound_matched = FALSE;
            target_sound_direction_valid = FALSE;
        }

        /* --- S3: 音源追従コントローラを先に実行し、目標操舵角を取得 --- */
        /* slot変更で旧見本が無効になった場合も、未学習のDoA/VADだけで発進しない。 */
        BOOL const match_required = (0U != CPU0_SOUND_REQUIRE_IDENTIFIER_MATCH);
        /* 見本一致の保持は走行継続用とし、別音のDoAを位置推定へ混ぜない。 */
        BOOL const target_sound_valid = link_ready &&
            (!match_required || (target_sound_matched && target_sound_direction_valid)) &&
            (snapshot.observation.doa_confidence >= CPU0_SOUND_LOCALIZATION_MIN_CONFIDENCE) &&
            (0U != snapshot.observation.vad) &&
            (snapshot.observation.level_dbfs_x100 >= CPU0_SOUND_TRIGGER_DBFS_X100);

        odometry_pose_t pose;
        memset(&pose, 0, sizeof(pose));
        odometry_service_get_pose(&pose);
        BOOL const pose_valid = sensor_fresh && pose.valid &&
            (0U != (sensor_snapshot.valid_flags & CPU0_SENSOR_VALID_IMU));
        sound_source_localizer_input_t localizer_input = {
            .pose = pose,
            .relative_doa_deg = observation_usable ?
                sound_follow_doa_to_relative(snapshot.observation.doa_deg) : 0,
            .doa_confidence = snapshot.observation.doa_confidence,
            .new_observation = new_observation,
            .sound_valid = target_sound_valid,
            .pose_valid = pose_valid,
            .observation_sequence = snapshot.observation_sequence,
            .now_ms = sensor_now_ms,
        };
        sound_source_localizer_output_t localizer_output;
        memset(&localizer_output, 0, sizeof(localizer_output));
        sound_source_localizer_step(&localizer_input, &localizer_output);

        g_task_think_raw_doa_deg = observation_usable ? snapshot.observation.raw_doa_deg :
                                                       ACOUSTIC_PROTOCOL_DOA_INVALID;
        g_task_think_filtered_doa_deg = observation_usable ? snapshot.observation.doa_deg :
                                                            ACOUSTIC_PROTOCOL_DOA_INVALID;
        g_task_think_doa_confidence = observation_usable ? snapshot.observation.doa_confidence : 0U;
        g_task_think_rover_x_mm = pose.x_mm;
        g_task_think_rover_y_mm = pose.y_mm;
        g_task_think_rover_heading_mrad = pose.theta_mrad;
        g_task_think_source_x_mm = localizer_output.source_x_mm;
        g_task_think_source_y_mm = localizer_output.source_y_mm;
        g_task_think_source_range_mm = localizer_output.source_range_mm;
        g_task_think_source_bearing_deg = localizer_output.source_bearing_deg;
        g_task_think_source_confidence = localizer_output.source_confidence;
        g_task_think_localization_observation_count = localizer_output.observation_count;
        g_task_think_localization_residual_mm = localizer_output.localization_residual_mm;
        g_task_think_localization_crossing_deg = localizer_output.bearing_crossing_angle_deg;
        g_task_think_localization_baseline_mm = localizer_output.baseline_mm;
        g_task_think_source_position_shift_mm = localizer_output.source_position_shift_mm;
        g_task_think_localization_geometry_valid = localizer_output.localization_geometry_valid;
        g_task_think_source_position_valid = localizer_output.source_position_valid;
        g_task_think_navigation_target_valid = localizer_output.navigation_target_valid;
        g_task_think_arrival_candidate = localizer_output.arrival_candidate;
        g_task_think_arrival_state = localizer_output.arrival_state;
        g_task_think_arrival_confirm_count = localizer_output.arrival_confirm_count;

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
            .target_sound_direction_valid = target_sound_direction_valid,
            .navigation_target_valid = (0U != CPU0_SOUND_USE_LOCALIZATION_FOR_STEERING) &&
                                       localizer_output.navigation_target_valid,
            .navigation_bearing_deg = localizer_output.source_bearing_deg,
            .arrival_verify = (0U != CPU0_SOUND_USE_LOCALIZATION_FOR_ARRIVAL) &&
                              (CPU0_SOUND_ARRIVAL_VERIFY == localizer_output.arrival_state),
            .arrived = (0U != CPU0_SOUND_USE_LOCALIZATION_FOR_ARRIVAL) &&
                       (CPU0_SOUND_ARRIVAL_ARRIVED == localizer_output.arrival_state),
            .avoidance_relisten = FALSE,
            .restart_request = restart_request,
            .imu_valid = sensor_fresh &&
                         (0U != (sensor_snapshot.valid_flags & CPU0_SENSOR_VALID_IMU)),
            .gyro_z_dps_x10 = sensor_snapshot.gyro_dps_x10[CPU0_SENSOR_YAW_AXIS],
            .imu_update_count = sensor_snapshot.update_count,
            .pose_heading_valid = pose_valid,
            .pose_heading_mrad = pose.theta_mrad,
            .rear_seam_turn_preference = sensor_fresh ?
                obstacle_avoidance_rear_seam_turn_preference(&sensor_snapshot) : 0,
        };
        /* 音源追従コントローラの出力を回避制御へ引き渡す領域。 */
        LOCAL sound_follow_output_t sf_output;
        memset(&sf_output, 0, sizeof(sf_output));
        sound_follow_state_t const previous_state = g_task_think_state;
        sound_follow_controller_step(&input, CPU0_THINK_PERIOD_MS, &sf_output);

        /* 現在の既定制御では位置推定を操舵に使わないため、ナビゲーション診断の方位欄へ
         * 静止中に確定したDoAを出す。最終操舵（ToF回避後）とは別に記録されるため、
         * 走行ログで音源方位の誤認と障害物回避の上書きを判別できる。 */
        if ((CPU0_THINK_STATE_STEER_PREP == sf_output.state) ||
            (CPU0_THINK_STATE_MOVE_STEP == sf_output.state) ||
            (CPU0_THINK_STATE_SPIN_PREP == sf_output.state) ||
            (CPU0_THINK_STATE_SPIN_STEP == sf_output.state)) {
            g_task_think_source_bearing_deg = sf_output.target_bearing_deg;
        }

        /* --- S3: 音源追従中だけ回避を開始し、開始済みなら通過まで保持する --- */
        BOOL const spin_requested = (CPU0_THINK_STATE_SPIN_PREP == sf_output.state) ||
                                    (CPU0_THINK_STATE_SPIN_STEP == sf_output.state);
        BOOL const sound_motion_active = (CPU0_THINK_STATE_MOVE_STEP == sf_output.state) ||
                                         spin_requested;
        if (match_required && !target_sound_matched) {
            /* 音源消失後に回避ラッチだけで走り続けない。 */
            avoidance_motion_latched = FALSE;
        }
        BOOL const obstacle_motion_context = sound_motion_active || avoidance_motion_latched;
        /* ToF回避コントローラの判断結果を安全調停へ渡す領域。 */
        LOCAL obstacle_avoidance_output_t oa_output;
        memset(&oa_output, 0, sizeof(oa_output));
        /* 回避がMOVE_STEPを越えて続く間も、直前の停止聴取で確定した音源側を
         * 脱出方向の同点判定に使う。SETTLE/LISTEN中のsteering_degは0へ戻るため、
         * ここでそれを渡すとピボットが音源方位を失う。 */
        H obstacle_target_steering_deg = sf_output.target_bearing_deg;
        if (obstacle_target_steering_deg > CPU0_SOUND_STEERING_MAX_DEG) {
            obstacle_target_steering_deg = CPU0_SOUND_STEERING_MAX_DEG;
        } else if (obstacle_target_steering_deg < -CPU0_SOUND_STEERING_MAX_DEG) {
            obstacle_target_steering_deg = -CPU0_SOUND_STEERING_MAX_DEG;
        }
        BOOL const spin_space_available = spin_requested && sensor_fresh &&
            (APP_FAULT_NONE == g_task_think_fault_flags) && !g_task_think_learning_mode &&
            obstacle_avoidance_spin_space_available(&sensor_snapshot);
        if (spin_space_available) {
            /* 前進回避は旋回の代わりにならない。安全な空間では音源への
             * IMU閉ループ旋回を通し、以前の前進回避ラッチは破棄する。 */
            obstacle_avoidance_controller_init();
            avoidance_motion_latched = FALSE;
            oa_output.state = sf_output.state;
            oa_output.rule = CPU0_SENSOR_RULE_FORWARD;
            oa_output.actuator_enable = TRUE;
        } else {
            actuator_status_t actuator_feedback = {0};
            BOOL const feedback_valid = sensor_fresh &&
                actuator_ipc_client_status_get(&actuator_feedback);
            obstacle_avoidance_encoder_feedback_set(feedback_valid, actuator_feedback.sequence_number,
                actuator_feedback.left_encoder_rpm_x10, actuator_feedback.right_encoder_rpm_x10);
            obstacle_avoidance_controller_step(sensor_fresh ? &sensor_snapshot : NULL,
                (APP_FAULT_NONE != g_task_think_fault_flags) || g_task_think_learning_mode ||
                    !obstacle_motion_context,
                sensor_now_ms, obstacle_target_steering_deg, pose.linear_speed_mm_s, &oa_output);
        }
        g_task_think_sensor_rule = oa_output.rule;

        if (sound_motion_active && oa_output.avoidance_in_progress) {
            avoidance_motion_latched = TRUE;
        }
        if (oa_output.avoidance_completed || (CPU0_SENSOR_RULE_BLOCKED_STOP == oa_output.rule) ||
            (CPU0_SENSOR_RULE_IMU_STOP == oa_output.rule)) {
            avoidance_motion_latched = FALSE;
        }
        if (oa_output.avoidance_completed) {
            /* 回避を完了した周期に必ず停止し、回避前・回避中のDoAを破棄する。 */
            input.avoidance_relisten = TRUE;
            input.restart_request = FALSE;
            sound_follow_controller_step(&input, 0U, &sf_output);
        }

#if (CPU0_USE_CONTROL_MLP != 0U)
        LOCAL control_mlp_output_t mlp_output;
        memset(&mlp_output, 0, sizeof(mlp_output));
        control_mlp_planner_step(sensor_fresh ? &sensor_snapshot : NULL,
                                 (float) sf_output.target_bearing_deg,
                                 &mlp_output);
#endif

        /* --- S4: 連続走行マージ（音源引力とToF斥力を合成した回避走行指令を採用） --- */
        /* 追従・回避・停止の優先順位を反映した最終要求値。 */
        LOCAL sound_follow_output_t output;
        memset(&output, 0, sizeof(output));
        BOOL const tracking_active = (CPU0_THINK_STATE_MOVE_STEP == sf_output.state);
        BOOL spin_active = (CPU0_THINK_STATE_SPIN_STEP == sf_output.state);
        BOOL const arrival_stop = (CPU0_THINK_STATE_ARRIVAL_VERIFY == sf_output.state) ||
                                  (CPU0_THINK_STATE_ARRIVED == sf_output.state);
        BOOL const avoidance_active = obstacle_motion_context && oa_output.avoidance_in_progress;
        BOOL const sensor_stop = obstacle_motion_context &&
                                 ((oa_output.rule == CPU0_SENSOR_RULE_SAFE_STOP) ||
                                  (oa_output.rule == CPU0_SENSOR_RULE_BLOCKED_STOP) ||
                                  (oa_output.rule == CPU0_SENSOR_RULE_IMU_STOP));

#if (CPU0_USE_CONTROL_MLP != 0U)
        BOOL const mlp_eligible = ((0U == CPU0_SOUND_STOP_AND_LISTEN_ENABLE) ||
                                   (0U != CPU0_SOUND_ALLOW_CONTROL_MLP)) &&
                                  (tracking_active || (CPU0_THINK_STATE_STEER_PREP == sf_output.state));
        if (mlp_eligible && !mlp_output.fallback_required && !avoidance_active && !sensor_stop && !arrival_stop &&
            !oa_output.emergency_stop && !sf_output.emergency_stop &&
            !g_task_think_learning_mode && (APP_FAULT_NONE == g_task_think_fault_flags)) {
            /* TFLM 制御MLP による連続回避・追従計画 */
            output.state = sf_output.state;
            output.steering_deg = (H) roundf(mlp_output.steering_deg);
            output.is_spin_turn = FALSE;
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
        } else if (sensor_stop) {
            /* センサー停止を到着停止で覆い隠さず、停止理由を独立して保持する。 */
            output.state = oa_output.state;
            output.steering_deg = 0;
            output.is_spin_turn = FALSE;
            output.left_rpm = 0;
            output.right_rpm = 0;
            output.actuator_enable = FALSE;
            output.emergency_stop = FALSE;
        } else if (arrival_stop) {
            /* 音源到着停止は障害物停止・非常停止と別状態で保持する。 */
            output = sf_output;
            output.left_rpm = 0;
            output.right_rpm = 0;
        } else if (avoidance_active && oa_output.actuator_enable) {
            /* 障害物回避の最小並進旋回を最優先実行 */
            output.state           = oa_output.state;
            output.steering_deg    = oa_output.steering_deg;
            output.is_spin_turn    = oa_output.is_spin_turn;
            output.left_rpm        = oa_output.left_rpm;
            output.right_rpm       = oa_output.right_rpm;
            output.actuator_enable = TRUE;
            output.emergency_stop  = FALSE;
        } else if (tracking_active && oa_output.actuator_enable) {
            /* S4連続走行: 音源引力とToF斥力を合成した回避走行指令を採用 */
            output.state           = oa_output.state;
            output.steering_deg    = oa_output.steering_deg;
            output.is_spin_turn    = FALSE;
            output.left_rpm        = oa_output.left_rpm;
            output.right_rpm       = oa_output.right_rpm;
            output.actuator_enable = TRUE;
            output.emergency_stop  = FALSE;
        } else if (spin_active && oa_output.actuator_enable) {
            /* 最小並進回頭は回避走行の操舵と混合せず、独立した左右逆回転を維持する。 */
            output = sf_output;
        } else if ((CPU0_THINK_STATE_STEER_PREP == sf_output.state) ||
                   (CPU0_THINK_STATE_SPIN_PREP == sf_output.state)) {
            /* 初回音源検知時は停車したまま操舵を整定する。 */
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

        /* --- スピンターン始動トルク探索＆定常回転維持（Breakaway & Hold）制御 --- */
        if (output.is_spin_turn) {
            BOOL const moving = (output.left_rpm != 0) || (output.right_rpm != 0);
            H const gyro_z = sensor_fresh ? sensor_snapshot.gyro_dps_x10[CPU0_SENSOR_YAW_AXIS] : 0;
            if (moving && output.actuator_enable && !output.emergency_stop) {
                BOOL const turn_left = (output.left_rpm < output.right_rpm) ||
                                       ((output.right_rpm > 0) && (output.left_rpm <= 0));
                H const spin_rpm = task_think_spin_breakaway_rpm_update(TRUE, TRUE, gyro_z);
                output.left_rpm = turn_left ? (H) -spin_rpm : spin_rpm;
                output.right_rpm = turn_left ? spin_rpm : (H) -spin_rpm;
            } else {
                (void) task_think_spin_breakaway_rpm_update(FALSE, FALSE, 0);
                output.left_rpm = 0;
                output.right_rpm = 0;
            }
        } else {
            (void) task_think_spin_breakaway_rpm_update(FALSE, FALSE, 0);
        }

        if (g_task_think_learning_mode) {
            output.emergency_stop = TRUE;
            output.actuator_enable = FALSE;
            output.steering_deg = 0;
            output.left_rpm = 0;
            output.right_rpm = 0;
            task_think_learning_capture(&snapshot, observation_usable);
        }

#if (CPU0_DEBUG_MOTOR_RECORDING_ENABLE != 0U)
        /* TARGET識別結果は走行開始条件に使わない。音源追従や回避走行と競合しない
         * 静止聴取状態からのみ、正面550/側面380 mm以上で低速直進。毎50 ms再検査。 */
        BOOL const debug_clearance = debug_motor_recording_clearance(
            sensor_snapshot.tof_distance_mm[0], sensor_snapshot.tof_distance_mm[1],
            sensor_snapshot.tof_distance_mm[2], CPU0_SENSOR_ESCAPE_SIDE_MM, CPU0_SENSOR_ESCAPE_FRONT_MM);
        BOOL const debug_allowed = (CPU0_THINK_STATE_LISTEN == sf_output.state) &&
            (APP_FAULT_NONE == g_task_think_fault_flags) && !g_task_think_learning_mode &&
            !target_sound_matched && !avoidance_motion_latched &&
            (E_OK == snapshot_err) && snapshot.usb_configured && snapshot.hello_received &&
            !sf_output.emergency_stop && !oa_output.emergency_stop && debug_clearance &&
            safety_arbiter_motion_allowed(&sensor_snapshot, sensor_fresh, NULL);
        debug_motor_recording_step(&debug_motor_state, sw_event, debug_allowed, CPU0_THINK_PERIOD_MS);
        if (debug_motor_state.active) {
            output.state = CPU0_THINK_STATE_SENSOR_FORWARD;
            output.steering_deg = 0;
            output.is_spin_turn = FALSE;
            output.left_rpm = CPU0_SENSOR_MIN_FORWARD_RPM;
            output.right_rpm = CPU0_SENSOR_MIN_FORWARD_RPM;
            output.actuator_enable = TRUE;
            output.emergency_stop = FALSE;
            g_task_think_sensor_rule = CPU0_SENSOR_RULE_FORWARD;
        }
        g_task_think_debug_motor_active = debug_motor_state.active;
        g_task_think_debug_motor_remaining_ms = debug_motor_state.remaining_ms;
#endif

        g_task_think_state = output.state;
        if (previous_state != g_task_think_state) {
            state_elapsed_ms = 0U;
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
        if ((output.left_rpm < 0) && (output.right_rpm < 0)) {
            g_task_think_autonomous_backup_count++;
        }

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

        heartbeat_elapsed_ms += CPU0_THINK_PERIOD_MS;
        if (heartbeat_elapsed_ms >= CPU0_LED_HEARTBEAT_PERIOD_MS) {
            heartbeat_elapsed_ms = 0U;
        }
        if (APP_FAULT_NONE != g_task_think_fault_flags) {
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
    (void) task_think_spin_breakaway_rpm_update(FALSE, FALSE, 0);
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
        task_think_led_update(0U, fault_elapsed_ms % CPU0_LED_HEARTBEAT_PERIOD_MS, fault_elapsed_ms);
        fault_elapsed_ms += CPU0_THINK_PERIOD_MS;
        (void) tk_dly_tsk(CPU0_THINK_PERIOD_MS);
    }
}
