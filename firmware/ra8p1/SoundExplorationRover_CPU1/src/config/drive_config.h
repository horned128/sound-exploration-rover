/** =================================================================*
 * @file   drive_config.h
 * @brief  駆動系と車輪速度の校正設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_DRIVE_H
#define SEROV_CPU1_CONFIG_DRIVE_H

#define DRIVE_TARGET_RPM_MAX               (300)            /**< 駆動目標の最大[RPM] */
/* 2000 mm手押し10区間の左右共通推定 701.85 counts/rev を丸めた実機校正値。 */
#define WHEEL_ENCODER_COUNTS_PER_REV       (702U)           /**< 車輪エンコーダカウント数の回転 */
#define WHEEL_SPEED_SAMPLE_PERIOD_MS       (100U)           /**< 車輪速度サンプルの周期[ms] */
#define WHEEL_ENCODER_LEFT_FORWARD_SIGN    (-1)             /**< 車輪エンコーダ左前進の符号 */
#define WHEEL_ENCODER_RIGHT_FORWARD_SIGN   (+1)             /**< 車輪エンコーダ右前進の符号 */

/* 正RPMは車体前進。左はLPWM、右はRPWMへ出力する。 */
#define DRIVE_CHASSIS_FORWARD_SIGN         (-1)             /**< 駆動車体前進の符号 */
#define DRIVE_LEFT_MOUNT_SIGN              (+1)             /**< 駆動左取付の符号 */
#define DRIVE_RIGHT_MOUNT_SIGN             (-1)             /**< 駆動右取付の符号 */
#define DRIVE_LEFT_FORWARD_SIGN            /**< 左側車輪の前進方向符号 */ \
    (DRIVE_CHASSIS_FORWARD_SIGN * DRIVE_LEFT_MOUNT_SIGN)
#define DRIVE_RIGHT_FORWARD_SIGN           /**< 右側車輪の前進方向符号 */ \
    (DRIVE_CHASSIS_FORWARD_SIGN * DRIVE_RIGHT_MOUNT_SIGN)
#define DRIVE_DUTY_MIN_PERMILLE            (0)              /**< 駆動デューティの最小[‰] */
#define DRIVE_DUTY_MAX_PERMILLE            (700)            /**< 駆動デューティの最大[‰] */
#define DRIVE_RAMP_PER_MS                  (2)              /**< 駆動の変化率[ms] */
#define DRIVE_UPDATE_PERIOD_MS             (5U)             /**< 駆動更新の周期[ms] */
#define DRIVE_LEFT_DUTY_SCALE_PERMILLE     (800U)           /**< 駆動左デューティのスケール[‰] */
#define DRIVE_RIGHT_DUTY_SCALE_PERMILLE    (800U)           /**< 駆動右デューティのスケール[‰] */
#define DRIVE_SPEED_FEEDBACK_ENABLE        (0U)             /**< 駆動速度帰還の有効化 */
#define DRIVE_SPEED_FEEDBACK_START_DELAY_MS (150U)          /**< 駆動速度帰還開始の遅延[ms] */
#define DRIVE_SPEED_FEEDBACK_KP_PERMILLE_PER_RPM (2)        /**< 駆動速度帰還の比例ゲイン[‰][RPM] */
#define DRIVE_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE (250)  /**< 駆動速度帰還最大の補正[‰] */

/*
 * 実機計測用の一時機能。計測完了後は必ず0へ戻して再ビルドする。
 * 通常のIPC指令、指令timeout、safe stopはこの機能でも有効に保つ。
 */
#define DRIVE_MEASUREMENT_TEST_ENABLE      (0U)             /**< 駆動計測試験の有効化 */
#define DRIVE_MEASUREMENT_MAX_DUTY_PERMILLE (560U)          /**< 駆動計測最大のデューティ[‰] */
#define DRIVE_MEASUREMENT_MODE_NORMAL      (0U)             /**< 速度計測の通常モード */
#define DRIVE_MEASUREMENT_MODE_DUTY_OVERRIDE (1U)           /**< 速度計測のデューティ上書きモード */
#define DRIVE_MEASUREMENT_MODE_FORCE_STOP  (2U)             /**< 速度計測の強制停止モード */
#define DRIVE_MEASUREMENT_STATUS_DISABLED  (0U)             /**< 速度計測が無効な状態 */
#define DRIVE_MEASUREMENT_STATUS_ACTIVE    (1U)             /**< 速度計測を実行中の状態 */
#define DRIVE_MEASUREMENT_STATUS_FORCE_STOP (2U)            /**< 速度計測の強制停止状態 */
#define DRIVE_MEASUREMENT_STATUS_WAITING_FOR_ENABLE (3U)    /**< 速度計測の有効化待ち状態 */
#define DRIVE_MEASUREMENT_STATUS_INVALID   (4U)             /**< 速度計測状態値が不正な状態 */
#define DRIVE_MEASUREMENT_STATUS_DRIVER_ERROR (5U)          /**< 速度計測ドライバ異常状態 */

#endif /* SEROV_CPU1_CONFIG_DRIVE_H */
