/** =================================================================*
 * @file   sensor_config.h
 * @brief  CPU0センサーとI2C構成の設定値
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_SENSOR_H
#define SEROV_CPU0_CONFIG_SENSOR_H

#define CPU0_SENSOR_I2C_ENABLED            (1U)             /**< センサーI2Cの有効化 */
#define CPU0_SENSOR_PERIOD_MS              (50U)            /**< センサーの周期[ms] */
#define CPU0_SENSOR_RETRY_PERIOD_MS        (1000U)          /**< センサー再試行の周期[ms] */
#define CPU0_SENSOR_STALE_TIMEOUT_MS       (200U)           /**< センサー停滞の期限[ms] */
#define CPU0_SENSOR_I2C_TIMEOUT_MS         (50U)            /**< センサーI2Cの期限[ms] */
#define CPU0_TOF_DATA_READY_TIMEOUT_MS     (100U)           /**< ToFデータ準備完了の期限[ms] */

/* TCA9548Aは常に対象一チャネルのみを有効にする。 */
#define CPU0_TCA9548A_ADDRESS              (0x70U)          /**< TCA9548Aのアドレス */
#define CPU0_TCA9548A_CHANNEL_LEFT         (0U)             /**< TCA9548Aチャネルの左 */
#define CPU0_TCA9548A_CHANNEL_CENTER       (1U)             /**< TCA9548Aチャネルの中央 */
#define CPU0_TCA9548A_CHANNEL_RIGHT        (2U)             /**< TCA9548Aチャネルの右 */
#define CPU0_TCA9548A_CHANNEL_BMI270       (3U)             /**< TCA9548AチャネルのBMI270 */
#define CPU0_VL53L1X_ADDRESS               (0x29U)          /**< VL53L1Xのアドレス */
#define CPU0_TOF_MIN_VALID_MM              (40U)            /**< ToF最小の有効[mm] */
#define CPU0_TOF_MAX_VALID_MM              (4000U)          /**< ToF最大の有効[mm] */

#define CPU0_BMI270_RESET_DELAY_MS         (10U)            /**< BMI270リセットの遅延[ms] */
#define CPU0_BMI270_STARTUP_DELAY_MS       (50U)            /**< BMI270起動の遅延[ms] */
#define CPU0_BMI270_ACCEL_LSB_PER_G        (8192)           /**< BMI270加速度のlsb[G] */
#define CPU0_BMI270_GYRO_RANGE_DPS_X10     (5000)           /**< BMI270角速度のレンジ[0.1dps] */

#endif /* SEROV_CPU0_CONFIG_SENSOR_H */
