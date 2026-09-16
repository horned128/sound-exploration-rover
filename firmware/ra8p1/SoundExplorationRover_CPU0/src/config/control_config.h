/** =================================================================*
 * @file   control_config.h
 * @brief  CPU0の音源追従・障害物回避の調整値
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_CONTROL_H
#define SEROV_CPU0_CONFIG_CONTROL_H

#define CPU0_AUTONOMY_MODE_SOUND_FOLLOW    (0U)
#define CPU0_AUTONOMY_MODE_SENSOR_RULE     (1U)
#define CPU0_AUTONOMY_MODE                 (CPU0_AUTONOMY_MODE_SENSOR_RULE)

#define CPU0_SENSOR_CLEAR_DISTANCE_MM      (1000U)
#define CPU0_SENSOR_CAUTION_DISTANCE_MM    (700U)
#define CPU0_SENSOR_SIDE_DISTANCE_MM       (500U)
#define CPU0_SENSOR_PIVOT_DISTANCE_MM       (500U)
#define CPU0_SENSOR_PIVOT_CLEAR_DISTANCE_MM (900U)
#define CPU0_SENSOR_CRITICAL_STOP_DISTANCE_MM (150U)
/* 車体前端の突出と減速・測距遅延を見込み、後退で得た距離だけでは復帰しない。 */
#define CPU0_SENSOR_ESCAPE_SIDE_MM          (380U)
#define CPU0_SENSOR_ESCAPE_FRONT_MM         (550U)
#define CPU0_SENSOR_BACKUP_CLEAR_MM         (750U)
#define CPU0_SENSOR_BACKUP_GAIN_MM          (150U)
#define CPU0_SENSOR_SETTLE_MS               (400U)
#define CPU0_SENSOR_BACKUP_MIN_MS           (600U)
#define CPU0_SENSOR_BACKUP_MAX_MS           (2500U)
#define CPU0_SENSOR_PIVOT_MIN_MS            (600U)
#define CPU0_SENSOR_PIVOT_MAX_MS            (4500U)
#define CPU0_SENSOR_PIVOT_MIN_YAW_MDEG      (45000)
#define CPU0_SENSOR_PIVOT_PROGRESS_MDEG     (5000)
#define CPU0_SENSOR_PIVOT_PROGRESS_MS       (1200U)
#define CPU0_SENSOR_CLEAR_HOLD_MS           (300U)
#define CPU0_SENSOR_REARM_CLEAR_MS          (1000U)
#define CPU0_SENSOR_COMMIT_YAW_MDEG         (100000)
#define CPU0_SENSOR_COMMIT_STEERING_DEG     (30)
#define CPU0_SENSOR_COMMIT_MAX_MS           (10000U)
#define CPU0_SENSOR_ESCAPE_MAX_ATTEMPTS     (3U)
#define CPU0_SENSOR_MAX_STEP_MS             (500U)
/* 実機ログの左旋回でZ負、右旋回でZ正。IMUの取付を変えたら再確認する。 */
#define CPU0_SENSOR_YAW_AXIS                (2U)
#define CPU0_SENSOR_YAW_RIGHT_SIGN          (1)
#define CPU0_SENSOR_YAW_DEADBAND_DPS_X10    (30)
/*
 * 駆動系の実機校正値（始動境界 155‰、確実動作 200‰）に基づくRPM設定:
 * 実効Duty = RPM * (1000 / 300) * (800 / 1000) = RPM * 2.667‰
 * - 85 RPM  => 実効 227‰ (最低下限ガード: 200‰デッドバンドを確実に突破)
 * - 100 RPM => 実効 267‰ (注意減速時 / 後退時)
 * - 120 RPM => 実効 320‰ (通常前進 / ピボット外輪)
 */
#define CPU0_SENSOR_FORWARD_RPM            (120)
#define CPU0_SENSOR_CAUTION_RPM            (100)
#define CPU0_SENSOR_MIN_FORWARD_RPM        (85)
#define CPU0_SENSOR_BACKUP_RPM             (-100)
#define CPU0_SENSOR_PIVOT_OUTER_RPM        (120)
#define CPU0_SENSOR_PIVOT_INNER_RPM        (80)
#define CPU0_SENSOR_STEERING_MIN_DEG       (1)
#define CPU0_SENSOR_STEERING_MAX_DEG       (45)
#define CPU0_SENSOR_IMU_MAX_TILT_MG        (700)
#define CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG    (2400)
#define CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10   (2000)

#define CPU0_SOUND_DOA_ZERO_OFFSET_DEG     (0)
#define CPU0_SOUND_DOA_CLOCKWISE_POSITIVE  (0U)
#define CPU0_SOUND_TRIGGER_DBFS_X100       (-4500)
#define CPU0_SOUND_RELEASE_DBFS_X100       (-4800)
#define CPU0_SOUND_DOA_SETTLE_MS           (500U)
#define CPU0_SOUND_DOA_ACQUIRE_TIMEOUT_MS  (2000U)
#define CPU0_SOUND_DOA_SAMPLE_COUNT        (5U)
#define CPU0_SOUND_DOA_STABLE_WIDTH_DEG    (20)
#define CPU0_SOUND_FRONT_TOLERANCE_DEG     (0)
#define CPU0_SOUND_STEERING_MIN_DEG        (1)
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
#define CPU0_STEERING_SERVO_OUTPUT_SIGN    (-1)

#endif /* SEROV_CPU0_CONFIG_CONTROL_H */
