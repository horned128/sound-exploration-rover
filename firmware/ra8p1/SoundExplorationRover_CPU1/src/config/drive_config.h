/** =================================================================*
 * @file   drive_config.h
 * @brief  駆動系と車輪速度の校正設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_DRIVE_H
#define SEROV_CPU1_CONFIG_DRIVE_H

#define DRIVE_TARGET_RPM_MAX               (300)
#define WHEEL_ENCODER_COUNTS_PER_REV       (900U)
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

#endif /* SEROV_CPU1_CONFIG_DRIVE_H */
