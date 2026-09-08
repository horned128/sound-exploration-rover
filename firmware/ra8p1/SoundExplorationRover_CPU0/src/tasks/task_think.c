/** =================================================================*
 * @file   task_think.c
 * @brief  CPU0音源追従思考タスク
 * ================================================================= */
#include "task_think.h"                                       /* CPU0思考タスクAPI */
#include "config/control_config.h"                         /* 思考モードと走行値 */
#include "config/pin_config.h"                             /* LEDの役割設定 */
#include "config/sensor_config.h"                          /* センサー安全判定値 */
#include "config/task_config.h"                            /* 思考周期と優先度 */
#include "control/obstacle_avoidance_controller.h"         /* ToF・IMU走行判断 */
#include "control/sound_follow_controller.h"                /* 音源追従状態機械 */
#include "hal_data.h"                                       /* BSP LED情報、ピンAPI */
#include "task_acoustic_link.h"                                       /* 最新音響状態取得API */
#include "task_command.h"                                     /* 最新アクチュエータ目標更新API */
#include "task_sensor.h"                                      /* 最新I2Cセンサー状態取得API */

IMPORT bsp_leds_t g_bsp_leds;                               /**< BSPのLED構成情報 */

LOCAL void task_think_entry(INT stacd, void * exinf);      /* 思考タスク本体 */
LOCAL ER task_think_publish_target(const sound_follow_output_t * p_output); /* 追従指令の4輪展開 */
LOCAL ER task_think_publish_motion(H steering_deg, H left_rpm, H right_rpm, BOOL actuator_enable,
                                   BOOL emergency_stop); /* 共通走行指令の4輪展開 */
LOCAL BOOL task_think_sound_motion_allowed(const sensor_snapshot_t * p_snapshot); /* 音源追従の近接安全判定 */
LOCAL void task_think_led_write(BOOL blue_on, BOOL green_on); /* 2LED一括更新 */
LOCAL UW task_think_fault_code(UW fault_flags);/* LED表示用異常番号 */
/* 状態LED更新 */
LOCAL void task_think_led_update(UW state_elapsed_ms, UW heartbeat_elapsed_ms, UW fault_elapsed_ms);

/**< 他タスクからの異常通知を集約するイベントフラグ設定 */
LOCAL T_CFLG const think_fault_flag_config = {
    .flgatr = TA_TFIFO | TA_WSGL,
    .iflgptn = 0U,
};

/**< 音源追従判断を行う思考タスク設定 */
LOCAL T_CTSK const think_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) task_think_entry,
    .itskpri = CPU0_THINK_TASK_PRIORITY,
    .stksz = CPU0_THINK_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID think_task_id;                                    /**< 思考タスクID */
LOCAL ID think_fault_flag_id;                              /**< CPU0異常イベントフラグID */
LOCAL BOOL think_task_started;                             /**< 思考タスク開始状態 */

EXPORT volatile sound_follow_state_t g_task_think_state;             /**< 現在の思考状態 */
EXPORT volatile UW g_task_think_cycle_count;                 /**< 思考周期実行回数 */
EXPORT volatile UW g_task_think_observation_sequence;        /**< 最終判断観測sequence */
EXPORT volatile UW g_task_think_observation_watchdog_ms;     /**< 観測更新停止時間 */
EXPORT volatile BOOL g_task_think_link_ready;                      /**< 音響リンク判断 */
EXPORT volatile BOOL g_task_think_new_observation;                 /**< 新規観測判断 */
EXPORT volatile H g_task_think_steering_deg;                 /**< 操舵判断値 */
EXPORT volatile H g_task_think_left_rpm;                     /**< 左RPM判断値 */
EXPORT volatile H g_task_think_right_rpm;                    /**< 右RPM判断値 */
EXPORT volatile BOOL g_task_think_actuator_enable;                 /**< 出力許可判断 */
EXPORT volatile BOOL g_task_think_emergency_stop;                  /**< 非常停止判断 */
EXPORT volatile obstacle_avoidance_rule_t g_task_think_sensor_rule;       /**< 選択センサー走行ルール */
EXPORT volatile UW g_task_think_fault_flags;                       /**< CPU0異常ラッチ */

/** =================================================================*
 * @brief  音源追従で前進してよいToF状態か判定
 * @details センサー取得失敗、更新期限超過、ToF無効、またはいずれかの測距が
 *          ハード停止距離未満なら走行を許可しない。
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 3台のToFが有効かつ近接障害物なしならtrue
 * ================================================================= */
LOCAL BOOL task_think_sound_motion_allowed(const sensor_snapshot_t * p_snapshot) {
    UB const required_tof_flags = CPU0_SENSOR_VALID_TOF_LEFT | CPU0_SENSOR_VALID_TOF_CENTER |
                                  CPU0_SENSOR_VALID_TOF_RIGHT;
    if ((NULL == p_snapshot) || !p_snapshot->initialized ||
        (required_tof_flags != (p_snapshot->valid_flags & required_tof_flags)) ||
        (p_snapshot->age_ms > CPU0_SENSOR_STALE_TIMEOUT_MS)) {
        return FALSE;
    }

    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        if (p_snapshot->tof_distance_mm[index] < CPU0_SENSOR_HARD_STOP_DISTANCE_MM) {
            return FALSE;
        }
    }
    return TRUE;
}

/** =================================================================*
 * @brief  思考タスクと異常イベント生成
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_think_create(void) {
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
    sound_follow_controller_init();
#if (CPU0_SENSOR_I2C_ENABLED != 0U)
    obstacle_avoidance_controller_init();
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

    task_think_led_write(blue_on, green_on);
}

/** =================================================================*
 * @brief  思考タスク本体
 * ================================================================= */
LOCAL void task_think_entry(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    UW state_elapsed_ms = 0U;
    UW heartbeat_elapsed_ms = 0U;
    UW fault_elapsed_ms = 0U;
#if (CPU0_AUTONOMY_MODE == CPU0_AUTONOMY_MODE_SOUND_FOLLOW)
    UW last_observation_sequence = 0U;
    BOOL observation_sequence_valid = FALSE;
#endif

    while (1) {
        UINT fault_pattern = 0U;
        ER const flag_err =
            tk_wai_flg(think_fault_flag_id, APP_FAULT_ALL_MASK, TWF_ORW | TWF_BITCLR, &fault_pattern, TMO_POL);
        if (E_OK == flag_err) {
            g_task_think_fault_flags |= fault_pattern;
        }

#if (CPU0_AUTONOMY_MODE == CPU0_AUTONOMY_MODE_SENSOR_RULE)
        sensor_snapshot_t sensor_snapshot = {0};
        ER const sensor_snapshot_err = task_sensor_snapshot_get(&sensor_snapshot);
        obstacle_avoidance_output_t output;
        sound_follow_state_t const previous_state = g_task_think_state;
        obstacle_avoidance_controller_step((E_OK == sensor_snapshot_err) ? &sensor_snapshot : NULL,
                                           APP_FAULT_NONE != g_task_think_fault_flags, &output);
        g_task_think_state = output.state;
        if (previous_state != g_task_think_state) {
            state_elapsed_ms = 0U;
        }

        if (E_OK != task_think_publish_motion(output.steering_deg, output.left_rpm, output.right_rpm,
                                               output.actuator_enable, output.emergency_stop)) {
            g_task_think_fault_flags |= APP_FAULT_TARGET_UPDATE;
            obstacle_avoidance_controller_step(NULL, TRUE, &output);
            g_task_think_state = output.state;
            (void) task_think_publish_motion(output.steering_deg, output.left_rpm, output.right_rpm,
                                              output.actuator_enable, output.emergency_stop);
        }

        g_task_think_link_ready = (E_OK == sensor_snapshot_err) && sensor_snapshot.initialized &&
                                  (CPU0_SENSOR_VALID_ALL == sensor_snapshot.valid_flags) &&
                                  (sensor_snapshot.age_ms <= CPU0_SENSOR_STALE_TIMEOUT_MS);
        g_task_think_new_observation = FALSE;
        g_task_think_steering_deg = output.steering_deg;
        g_task_think_left_rpm = output.left_rpm;
        g_task_think_right_rpm = output.right_rpm;
        g_task_think_actuator_enable = output.actuator_enable;
        g_task_think_emergency_stop = output.emergency_stop;
        g_task_think_sensor_rule = output.rule;
#else
        task_acoustic_link_snapshot_t snapshot = {0};
        sensor_snapshot_t sensor_snapshot = {0};
        ER const snapshot_err = task_acoustic_link_snapshot_get(&snapshot);
        ER const sensor_snapshot_err = task_sensor_snapshot_get(&sensor_snapshot);
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

        sound_follow_input_t input = {
            .link_ready = link_ready,
            .new_observation = new_observation,
            .fault_active = APP_FAULT_NONE != g_task_think_fault_flags,
            .motion_allowed = (E_OK == sensor_snapshot_err) && task_think_sound_motion_allowed(&sensor_snapshot),
            .observation = snapshot.observation,
        };
        sound_follow_output_t output;
        sound_follow_state_t const previous_state = g_task_think_state;
        sound_follow_controller_step(&input, CPU0_THINK_PERIOD_MS, &output);
        g_task_think_state = output.state;
        if (previous_state != g_task_think_state) {
            state_elapsed_ms = 0U;
        }

        if (E_OK != task_think_publish_target(&output)) {
            g_task_think_fault_flags |= APP_FAULT_TARGET_UPDATE;
            input.fault_active = TRUE;
            sound_follow_controller_step(&input, 0U, &output);
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
#endif

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
