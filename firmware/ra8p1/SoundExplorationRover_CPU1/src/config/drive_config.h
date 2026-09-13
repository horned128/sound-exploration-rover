/** =================================================================*
 * @file   drive_config.h
 * @brief  駆動系と車輪速度の校正設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_DRIVE_H
#define SEROV_CPU1_CONFIG_DRIVE_H

#define DRIVE_TARGET_RPM_MAX               (300)
/* 2000 mm手押し10区間の左右共通推定 701.85 counts/rev を丸めた実機校正値。 */
#define WHEEL_ENCODER_COUNTS_PER_REV       (702U)
#define WHEEL_SPEED_SAMPLE_PERIOD_MS       (100U)
#define WHEEL_ENCODER_LEFT_FORWARD_SIGN    (-1)
#define WHEEL_ENCODER_RIGHT_FORWARD_SIGN   (+1)

/* 正RPMは車体前進。左はLPWM、右はRPWMへ出力する。 */
#define DRIVE_CHASSIS_FORWARD_SIGN         (-1)
#define DRIVE_LEFT_MOUNT_SIGN              (+1)
#define DRIVE_RIGHT_MOUNT_SIGN             (-1)
#define DRIVE_LEFT_FORWARD_SIGN            (DRIVE_CHASSIS_FORWARD_SIGN * DRIVE_LEFT_MOUNT_SIGN)
#define DRIVE_RIGHT_FORWARD_SIGN           (DRIVE_CHASSIS_FORWARD_SIGN * DRIVE_RIGHT_MOUNT_SIGN)
#define DRIVE_DUTY_MIN_PERMILLE            (0)
#define DRIVE_DUTY_MAX_PERMILLE            (700)
#define DRIVE_RAMP_PER_MS                  (2)
#define DRIVE_UPDATE_PERIOD_MS             (5U)
#define DRIVE_LEFT_DUTY_SCALE_PERMILLE     (800U)
#define DRIVE_RIGHT_DUTY_SCALE_PERMILLE    (800U)
#define DRIVE_SPEED_FEEDBACK_ENABLE        (0U)
#define DRIVE_SPEED_FEEDBACK_START_DELAY_MS (150U)
#define DRIVE_SPEED_FEEDBACK_KP_PERMILLE_PER_RPM (2)
#define DRIVE_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE (250)

/*
 * 実機計測用の一時機能。計測完了後は必ず0へ戻して再ビルドする。
 * 通常のIPC指令、指令timeout、safe stopはこの機能でも有効に保つ。
 */
#define DRIVE_MEASUREMENT_TEST_ENABLE       (0U)
#define DRIVE_MEASUREMENT_MAX_DUTY_PERMILLE (560U)
#define DRIVE_MEASUREMENT_MODE_NORMAL       (0U)
#define DRIVE_MEASUREMENT_MODE_DUTY_OVERRIDE (1U)
#define DRIVE_MEASUREMENT_MODE_FORCE_STOP   (2U)
#define DRIVE_MEASUREMENT_STATUS_DISABLED  (0U)
#define DRIVE_MEASUREMENT_STATUS_ACTIVE    (1U)
#define DRIVE_MEASUREMENT_STATUS_FORCE_STOP (2U)
#define DRIVE_MEASUREMENT_STATUS_WAITING_FOR_ENABLE (3U)
#define DRIVE_MEASUREMENT_STATUS_INVALID    (4U)
#define DRIVE_MEASUREMENT_STATUS_DRIVER_ERROR (5U)

#endif /* SEROV_CPU1_CONFIG_DRIVE_H */
