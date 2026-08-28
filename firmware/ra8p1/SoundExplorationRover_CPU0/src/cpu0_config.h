/** =================================================================*
 * @file   cpu0_config.h
 * @brief  CPU0動作設定
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_H
#define SEROV_CPU0_CONFIG_H

#define CPU0_ACTUATOR_STARTUP_DELAY_MS     (250U)
#define CPU0_COMMAND_PERIOD_MS             (50U)
#define CPU0_COMMAND_TARGET_TIMEOUT_MS     (500U)
#define CPU0_THINK_PERIOD_MS               (100U)
#define CPU0_AUDIO_USB_POLL_MS             (1U)
#define CPU0_AUDIO_USB_RX_SIZE             (512U)
#define CPU0_AUDIO_TELEMETRY_PERIOD_MS     (250U)
#define CPU0_IPC_RETRY_DELAY_MS            (1U)

/* 数値が小さいほど高優先度。IPC keep-aliveを思考処理より優先する。 */
#define CPU0_INIT_TASK_PRIORITY            (5)
#define CPU0_COMMAND_TASK_PRIORITY         (6)
#define CPU0_AUDIO_TASK_PRIORITY           (8)
#define CPU0_THINK_TASK_PRIORITY           (10)
#define CPU0_INIT_TASK_STACK_SIZE          (1024U)
#define CPU0_COMMAND_TASK_STACK_SIZE       (1024U)
#define CPU0_AUDIO_TASK_STACK_SIZE         (2048U)
#define CPU0_THINK_TASK_STACK_SIZE         (1024U)

/* ReSpeakerはESP32S3実装面を上にして搭載する。DoA原点と回転方向は車体上で校正する。 */
#define CPU0_SOUND_DOA_ZERO_OFFSET_DEG     (180)
#define CPU0_SOUND_DOA_CLOCKWISE_POSITIVE  (0U)
#define CPU0_SOUND_REQUIRE_VAD             (0U)
#define CPU0_SOUND_TRIGGER_DBFS_X100       (-4500)
#define CPU0_SOUND_RELEASE_DBFS_X100       (-4800)
#define CPU0_SOUND_TRIGGER_HOLD_MS         (300U)
#define CPU0_SOUND_DOA_SAMPLE_COUNT        (5U)
#define CPU0_SOUND_DOA_STABLE_WIDTH_DEG    (20)
#define CPU0_SOUND_FRONT_TOLERANCE_DEG     (15)
#define CPU0_SOUND_STEERING_MIN_DEG        (20)
#define CPU0_SOUND_STEERING_MAX_DEG        (45)
#define CPU0_SOUND_REVERSE_ANGLE_DEG       (100)
#define CPU0_SOUND_LINK_STABLE_MS          (500U)
#define CPU0_SOUND_OBSERVATION_TIMEOUT_MS  (600U)
#define CPU0_SOUND_STEER_SETTLE_MS         (500U)
#define CPU0_SOUND_MOVE_STEP_MS            (500U)
#define CPU0_SOUND_LISTEN_SETTLE_MS        (500U)
#define CPU0_SOUND_COOLDOWN_RELEASE_MS     (200U)
#define CPU0_SOUND_MOVE_LEFT_RPM           (120)
#define CPU0_SOUND_MOVE_RIGHT_RPM          (120)
#define CPU0_SOUND_TURN_INNER_RPM          (90)

/* 正の車体操舵値を実機の右旋回へ変換するサーボ出力の符号。 */
#define CPU0_STEERING_SERVO_OUTPUT_SIGN    (-1)

/* 青LED: 思考状態、緑LED: heartbeatまたはfault。赤LEDはCPU1専用。 */
#define CPU0_THINK_BLUE_LED_INDEX          (0U)
#define CPU0_THINK_GREEN_LED_INDEX         (1U)
#define CPU0_LED_HEARTBEAT_PERIOD_MS       (1000U)
#define CPU0_LED_HEARTBEAT_PULSE_MS        (100U)
#define CPU0_LED_WAIT_LINK_BLINK_MS        (500U)
#define CPU0_LED_LISTEN_BLINK_MS           (1000U)
#define CPU0_LED_STEER_BLINK_MS            (125U)
#define CPU0_LED_SETTLE_BLINK_MS           (250U)
#define CPU0_LED_FAULT_PULSE_MS            (100U)
#define CPU0_LED_FAULT_GAP_MS              (1000U)

#endif /* SEROV_CPU0_CONFIG_H */
