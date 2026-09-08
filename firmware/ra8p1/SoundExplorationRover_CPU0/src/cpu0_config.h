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
#define CPU0_IPC_SEND_RETRY_COUNT           (20U)

/* I2C1センサータスクを登録する。センサー未接続時は安全停止用の無効状態を公開する。 */
#define CPU0_SENSOR_I2C_ENABLED             (1U)
#define CPU0_SENSOR_PERIOD_MS                (50U)
#define CPU0_SENSOR_RETRY_PERIOD_MS          (1000U)
#define CPU0_SENSOR_STALE_TIMEOUT_MS         (200U)
#define CPU0_SENSOR_I2C_TIMEOUT_MS           (20U)
#define CPU0_TOF_DATA_READY_TIMEOUT_MS        (100U)

/*
 * TCA9548A配下のセンサチャネル割当。
 * 排他ルール: センサ通信の直前に対象チャネルだけをselectし、複数チャネルを同時に有効化しない。
 */
#define CPU0_TCA9548A_ADDRESS                (0x70U)
#define CPU0_TCA9548A_CHANNEL_LEFT           (0U)
#define CPU0_TCA9548A_CHANNEL_CENTER         (1U)
#define CPU0_TCA9548A_CHANNEL_RIGHT          (2U)
#define CPU0_TCA9548A_CHANNEL_BMI270         (3U)
#define CPU0_VL53L1X_ADDRESS                 (0x29U)
#define CPU0_TOF_MIN_VALID_MM                (40U)
#define CPU0_TOF_MAX_VALID_MM                (4000U)

/* BMI270は4g、500dps、100Hzでraw accel/gyroを取得する。 */
#define CPU0_BMI270_RESET_DELAY_MS           (10U)
#define CPU0_BMI270_STARTUP_DELAY_MS         (50U)
#define CPU0_BMI270_ACCEL_LSB_PER_G          (8192)
#define CPU0_BMI270_GYRO_RANGE_DPS_X10       (5000)

/* 自律モード。音源追従または距離・姿勢ルールを選択する。 */
#define CPU0_AUTONOMY_MODE_SOUND_FOLLOW      (0U)
#define CPU0_AUTONOMY_MODE_SENSOR_RULE       (1U)
#define CPU0_AUTONOMY_MODE                   (CPU0_AUTONOMY_MODE_SOUND_FOLLOW)

/* ルールベース走行の距離[mm]・速度[RPM]・操舵[deg]。 */
#define CPU0_SENSOR_CLEAR_DISTANCE_MM        (1000U)
#define CPU0_SENSOR_CAUTION_DISTANCE_MM      (650U)
#define CPU0_SENSOR_SIDE_DISTANCE_MM         (450U)
#define CPU0_SENSOR_HARD_STOP_DISTANCE_MM    (250U)
#define CPU0_SENSOR_BLOCKED_DISTANCE_MM      (350U)
#define CPU0_SENSOR_FORWARD_RPM              (55)
#define CPU0_SENSOR_CAUTION_RPM              (35)
#define CPU0_SENSOR_TURN_OUTER_RPM           (45)
#define CPU0_SENSOR_TURN_INNER_RPM           (25)
#define CPU0_SENSOR_TURN_STEERING_DEG        (35)

/* BMI270のZ軸を車体上向きとして取り付ける。傾き・衝撃時は停止する。 */
#define CPU0_SENSOR_IMU_MAX_TILT_MG          (700)
#define CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG      (2400)
#define CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10     (2000)

/* 数値が小さいほど高優先度。IPC keep-aliveを思考処理より優先する。 */
#define CPU0_COMMAND_TASK_PRIORITY         (6)
#define CPU0_AUDIO_TASK_PRIORITY           (8)
#define CPU0_SENSOR_TASK_PRIORITY          (9)
#define CPU0_THINK_TASK_PRIORITY           (10)
#define CPU0_COMMAND_TASK_STACK_SIZE       (1024U)
#define CPU0_AUDIO_TASK_STACK_SIZE         (2048U)
#define CPU0_SENSOR_TASK_STACK_SIZE        (2048U)
#define CPU0_THINK_TASK_STACK_SIZE         (1024U)

/* ReSpeakerはESP32S3実装面を上にして搭載する。DoA原点と回転方向は車体上で校正する。 */
#define CPU0_SOUND_DOA_ZERO_OFFSET_DEG     (0)
#define CPU0_SOUND_DOA_CLOCKWISE_POSITIVE  (0U)
#define CPU0_SOUND_TRIGGER_DBFS_X100       (-4500)
#define CPU0_SOUND_RELEASE_DBFS_X100       (-4800)
#define CPU0_SOUND_DOA_SETTLE_MS           (500U)
#define CPU0_SOUND_DOA_ACQUIRE_TIMEOUT_MS  (2000U)
#define CPU0_SOUND_DOA_SAMPLE_COUNT        (5U)
#define CPU0_SOUND_DOA_STABLE_WIDTH_DEG    (20)
#define CPU0_SOUND_FRONT_TOLERANCE_DEG     (15)
#define CPU0_SOUND_STEERING_MIN_DEG        (20)
#define CPU0_SOUND_STEERING_MAX_DEG        (45)
#define CPU0_SOUND_LINK_STABLE_MS          (500U)
#define CPU0_SOUND_OBSERVATION_TIMEOUT_MS  (600U)
#define CPU0_SOUND_STEER_SETTLE_MS         (500U)
#define CPU0_SOUND_MOVE_STEP_MS            (1000U)
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
