/** =================================================================*
 * @file   tk_think.c
 * @brief  音響観測から走行目標を生成するCPU0思考タスク
 * ================================================================= */
#include "tk_think.h"                                    /* CPU0思考タスクAPI */
#include "tk_audio.h"                                    /* 最新音響状態取得API */
#include "tk_command.h"                                  /* 最新アクチュエータ目標更新API */
#include "../control/sound_follow_controller.h"          /* 音源追従状態機械 */
#include "../../cpu0_config.h"                           /* 思考周期、LED設定 */
#include "hal_data.h"                                    /* BSP LED情報、ピンAPI */

extern bsp_leds_t g_bsp_leds;                             /**< BSPのLED構成情報 */

static void cpu0_think_task(INT stacd, void * exinf);      /* 思考タスク本体 */
static ER cpu0_think_publish_target(
    const sound_follow_output_t * p_output);               /* 追従指令の4輪展開 */
static void cpu0_think_led_write(
    bool blue_on,
    bool green_on);                                       /* 2LED一括更新 */
static uint32_t cpu0_think_fault_code(
    uint32_t fault_flags);                                /* LED表示用異常番号 */
static void cpu0_think_led_update(
    uint32_t state_elapsed_ms,
    uint32_t heartbeat_elapsed_ms,
    uint32_t fault_elapsed_ms);                           /* 状態LED更新 */

static T_CFLG const think_fault_flag_config = {
    .flgatr = TA_TFIFO | TA_WSGL,
    .iflgptn = 0U,
};

static T_CTSK const think_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) cpu0_think_task,
    .itskpri = CPU0_THINK_TASK_PRIORITY,
    .stksz = CPU0_THINK_TASK_STACK_SIZE,
    .bufptr = NULL,
};

static ID think_task_id;                                  /**< 思考タスクID */
static ID think_fault_flag_id;                            /**< CPU0異常イベントフラグID */
static bool think_task_started;                           /**< 思考タスク開始状態 */

volatile cpu0_think_state_t g_cpu0_think_state;          /**< 現在の思考状態 */
volatile uint32_t g_cpu0_think_cycle_count;               /**< 思考周期実行回数 */
volatile uint32_t g_cpu0_think_observation_sequence;      /**< 最終判断観測sequence */
volatile uint32_t g_cpu0_think_observation_watchdog_ms;   /**< 観測更新停止時間 */
volatile bool g_cpu0_think_link_ready;                    /**< 音響リンク判断 */
volatile bool g_cpu0_think_new_observation;               /**< 新規観測判断 */
volatile int16_t g_cpu0_think_steering_deg;               /**< 操舵判断値 */
volatile int16_t g_cpu0_think_left_rpm;                   /**< 左RPM判断値 */
volatile int16_t g_cpu0_think_right_rpm;                  /**< 右RPM判断値 */
volatile bool g_cpu0_think_actuator_enable;               /**< 出力許可判断 */
volatile bool g_cpu0_think_emergency_stop;                /**< 非常停止判断 */
volatile uint32_t g_cpu0_fault_flags;                     /**< CPU0異常ラッチ */

/** =================================================================*
 * @brief  思考タスクと異常イベント生成
 * @return CPU0異常コード
 * ================================================================= */
cpu0_fault_t cpu0_think_task_create(void) {
    think_task_id = 0;
    think_fault_flag_id = 0;
    think_task_started = false;
    g_cpu0_think_state = CPU0_THINK_STATE_WAIT_LINK;
    g_cpu0_think_cycle_count = 0U;
    g_cpu0_think_observation_sequence = 0U;
    g_cpu0_think_observation_watchdog_ms = UINT32_MAX;
    g_cpu0_think_link_ready = false;
    g_cpu0_think_new_observation = false;
    g_cpu0_think_steering_deg = 0;
    g_cpu0_think_left_rpm = 0;
    g_cpu0_think_right_rpm = 0;
    g_cpu0_think_actuator_enable = false;
    g_cpu0_think_emergency_stop = true;
    g_cpu0_fault_flags = CPU0_FAULT_NONE;
    sound_follow_controller_init();

    think_fault_flag_id = tk_cre_flg(&think_fault_flag_config);
    if (think_fault_flag_id <= 0) {
        think_fault_flag_id = 0;
        return CPU0_FAULT_TASK_CREATE;
    }

    think_task_id = tk_cre_tsk(&think_task_config);
    if (think_task_id <= 0) {
        think_task_id = 0;
        cpu0_think_task_delete();
        return CPU0_FAULT_TASK_CREATE;
    }

    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  思考タスク開始
 * @return CPU0異常コード
 * ================================================================= */
cpu0_fault_t cpu0_think_task_start(void) {
    if (think_task_id <= 0) {
        return CPU0_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(think_task_id, 0);
    if (E_OK != err) {
        return CPU0_FAULT_TASK_START;
    }
    think_task_started = true;
    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  思考タスクと異常イベント解放
 * ================================================================= */
void cpu0_think_task_delete(void) {
    if (think_task_id > 0) {
        if (think_task_started) {
            (void) tk_ter_tsk(think_task_id);
        }
        (void) tk_del_tsk(think_task_id);
        think_task_id = 0;
        think_task_started = false;
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
ER cpu0_think_report_fault(cpu0_fault_t fault) {
    if (CPU0_FAULT_NONE == fault) {
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
static ER cpu0_think_publish_target(
    const sound_follow_output_t * p_output) {
    if (NULL == p_output) {
        return E_PAR;
    }

    rover_motion_target_t target = {
        .left_target_rpm = p_output->left_rpm,
        .right_target_rpm = p_output->right_rpm,
        .actuator_enable = p_output->actuator_enable,
        .emergency_stop = p_output->emergency_stop,
    };

    int16_t const front_steering_deg = (int16_t) (
        CPU0_STEERING_SERVO_OUTPUT_SIGN * p_output->steering_deg);
    target.servo_target_deg[0] = front_steering_deg;       /* FR */
    target.servo_target_deg[1] = front_steering_deg;       /* FL */
    target.servo_target_deg[2] =
        (int16_t) -front_steering_deg;                     /* RR */
    target.servo_target_deg[3] =
        (int16_t) -front_steering_deg;                     /* RL */

    return cpu0_command_set_target(&target);
}

/** =================================================================*
 * @brief  青・緑LED一括更新
 * @param[in] blue_on 青LED点灯状態
 * @param[in] green_on 緑LED点灯状態
 * ================================================================= */
static void cpu0_think_led_write(bool blue_on, bool green_on) {
    bsp_leds_t const leds = g_bsp_leds;
    if (leds.led_count <= CPU0_THINK_GREEN_LED_INDEX) {
        return;
    }

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(
        (bsp_io_port_pin_t) leds.p_leds[CPU0_THINK_BLUE_LED_INDEX],
        blue_on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinWrite(
        (bsp_io_port_pin_t) leds.p_leds[CPU0_THINK_GREEN_LED_INDEX],
        green_on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();
}

/** =================================================================*
 * @brief  異常ビットを青LED点滅回数へ変換
 * @param[in] fault_flags CPU0異常ラッチ
 * @return 1～6の異常番号
 * ================================================================= */
static uint32_t cpu0_think_fault_code(uint32_t fault_flags) {
    if (0U != (fault_flags &
        (CPU0_FAULT_TASK_CREATE | CPU0_FAULT_TASK_START))) {
        return 1U;
    }
    if (0U != (fault_flags & CPU0_FAULT_IPC_INIT)) {
        return 2U;
    }
    if (0U != (fault_flags & CPU0_FAULT_IPC_SEND)) {
        return 3U;
    }
    if (0U != (fault_flags & CPU0_FAULT_COMMAND_TARGET_TIMEOUT)) {
        return 4U;
    }
    if (0U != (fault_flags & CPU0_FAULT_USB_INIT)) {
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
static void cpu0_think_led_update(uint32_t state_elapsed_ms,
                                  uint32_t heartbeat_elapsed_ms,
                                  uint32_t fault_elapsed_ms) {
    bool blue_on = false;
    bool green_on = false;

    if (CPU0_FAULT_NONE != g_cpu0_fault_flags) {
        uint32_t const code = cpu0_think_fault_code(
            g_cpu0_fault_flags);
        uint32_t const pulse_window_ms =
            code * CPU0_LED_FAULT_PULSE_MS * 2U;
        uint32_t const pattern_ms = pulse_window_ms +
                                    CPU0_LED_FAULT_GAP_MS;
        uint32_t const position_ms = fault_elapsed_ms % pattern_ms;

        green_on = true;
        blue_on = (position_ms < pulse_window_ms) &&
                  (0U == ((position_ms /
                           CPU0_LED_FAULT_PULSE_MS) & 1U));
    } else {
        green_on = heartbeat_elapsed_ms <
                   CPU0_LED_HEARTBEAT_PULSE_MS;

        switch (g_cpu0_think_state) {
            case CPU0_THINK_STATE_WAIT_LINK:
                green_on = false;
                blue_on = 0U == ((state_elapsed_ms /
                                  CPU0_LED_WAIT_LINK_BLINK_MS) & 1U);
                break;

            case CPU0_THINK_STATE_LISTEN:
                blue_on = state_elapsed_ms %
                          CPU0_LED_LISTEN_BLINK_MS <
                          CPU0_LED_HEARTBEAT_PULSE_MS;
                break;

            case CPU0_THINK_STATE_STEER_PREP:
                blue_on = 0U == ((state_elapsed_ms /
                                  CPU0_LED_STEER_BLINK_MS) & 1U);
                break;

            case CPU0_THINK_STATE_MOVE_STEP:
                blue_on = true;
                break;

            case CPU0_THINK_STATE_SETTLE:
            case CPU0_THINK_STATE_COOLDOWN:
                blue_on = 0U == ((state_elapsed_ms /
                                  CPU0_LED_SETTLE_BLINK_MS) & 1U);
                break;

            default:
                break;
        }
    }

    cpu0_think_led_write(blue_on, green_on);
}

/** =================================================================*
 * @brief  思考タスク本体
 * ================================================================= */
static void cpu0_think_task(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    uint32_t state_elapsed_ms = 0U;
    uint32_t heartbeat_elapsed_ms = 0U;
    uint32_t fault_elapsed_ms = 0U;
    uint32_t last_observation_sequence = 0U;
    bool observation_sequence_valid = false;

    while (1) {
        UINT fault_pattern = 0U;
        ER const flag_err = tk_wai_flg(think_fault_flag_id,
                                       CPU0_FAULT_ALL_MASK,
                                       TWF_ORW | TWF_BITCLR,
                                       &fault_pattern,
                                       TMO_POL);
        if (E_OK == flag_err) {
            g_cpu0_fault_flags |= fault_pattern;
        }

        cpu0_audio_snapshot_t snapshot = {0};
        ER const snapshot_err = cpu0_audio_snapshot_get(&snapshot);
        bool const observation_usable = (E_OK == snapshot_err) &&
            snapshot.usb_configured &&
            snapshot.hello_received &&
            snapshot.observation_received &&
            (ACOUSTIC_XVF_STATUS_READY ==
             snapshot.observation.xvf_status) &&
            (0U == (snapshot.observation.audio_flags &
                     (ACOUSTIC_AUDIO_FLAG_I2C_ERROR |
                      ACOUSTIC_AUDIO_FLAG_MUTED |
                      ACOUSTIC_AUDIO_FLAG_I2S_STALE)));
        bool const sequence_changed = observation_usable &&
            (!observation_sequence_valid ||
             (snapshot.observation_sequence !=
              last_observation_sequence));
        if (sequence_changed) {
            last_observation_sequence = snapshot.observation_sequence;
            g_cpu0_think_observation_sequence =
                snapshot.observation_sequence;
            g_cpu0_think_observation_watchdog_ms = 0U;
            observation_sequence_valid = true;
        } else if (observation_sequence_valid &&
                   (g_cpu0_think_observation_watchdog_ms <=
                    UINT32_MAX - CPU0_THINK_PERIOD_MS)) {
            g_cpu0_think_observation_watchdog_ms +=
                CPU0_THINK_PERIOD_MS;
        }

        bool const link_ready = observation_usable &&
            (snapshot.observation_age_ms <=
             CPU0_SOUND_OBSERVATION_TIMEOUT_MS) &&
            (g_cpu0_think_observation_watchdog_ms <
             CPU0_SOUND_OBSERVATION_TIMEOUT_MS);
        bool const new_observation = link_ready && sequence_changed;
        if (!observation_usable) {
            observation_sequence_valid = false;
            g_cpu0_think_observation_watchdog_ms = UINT32_MAX;
        }

        sound_follow_input_t input = {
            .link_ready = link_ready,
            .new_observation = new_observation,
            .fault_active = CPU0_FAULT_NONE != g_cpu0_fault_flags,
            .observation = snapshot.observation,
        };
        sound_follow_output_t output;
        cpu0_think_state_t const previous_state =
            g_cpu0_think_state;
        sound_follow_controller_step(&input,
                                     CPU0_THINK_PERIOD_MS,
                                     &output);
        g_cpu0_think_state = output.state;
        if (previous_state != g_cpu0_think_state) {
            state_elapsed_ms = 0U;
        }

        if (E_OK != cpu0_think_publish_target(&output)) {
            g_cpu0_fault_flags |= CPU0_FAULT_TARGET_UPDATE;
            input.fault_active = true;
            sound_follow_controller_step(&input, 0U, &output);
            g_cpu0_think_state = output.state;
            (void) cpu0_think_publish_target(&output);
        }

        g_cpu0_think_link_ready = link_ready;
        g_cpu0_think_new_observation = new_observation;
        g_cpu0_think_steering_deg = output.steering_deg;
        g_cpu0_think_left_rpm = output.left_rpm;
        g_cpu0_think_right_rpm = output.right_rpm;
        g_cpu0_think_actuator_enable = output.actuator_enable;
        g_cpu0_think_emergency_stop = output.emergency_stop;

        cpu0_think_led_update(state_elapsed_ms,
                              heartbeat_elapsed_ms,
                              fault_elapsed_ms);
        g_cpu0_think_cycle_count++;
        state_elapsed_ms += CPU0_THINK_PERIOD_MS;

        if (CPU0_FAULT_NONE == g_cpu0_fault_flags) {
            heartbeat_elapsed_ms += CPU0_THINK_PERIOD_MS;
            if (heartbeat_elapsed_ms >=
                CPU0_LED_HEARTBEAT_PERIOD_MS) {
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
void cpu0_think_halt(cpu0_fault_t fault) {
    uint32_t fault_elapsed_ms = 0U;
    g_cpu0_fault_flags |= (uint32_t) fault;
    g_cpu0_think_state = CPU0_THINK_STATE_FAULT;
    g_cpu0_think_link_ready = false;
    g_cpu0_think_new_observation = false;
    g_cpu0_think_steering_deg = 0;
    g_cpu0_think_left_rpm = 0;
    g_cpu0_think_right_rpm = 0;
    g_cpu0_think_actuator_enable = false;
    g_cpu0_think_emergency_stop = true;

    while (1) {
        cpu0_think_led_update(0U, 0U, fault_elapsed_ms);
        fault_elapsed_ms += CPU0_THINK_PERIOD_MS;
        (void) tk_dly_tsk(CPU0_THINK_PERIOD_MS);
    }
}
