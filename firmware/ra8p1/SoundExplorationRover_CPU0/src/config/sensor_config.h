/** =================================================================*
 * @file   sensor_config.h
 * @brief  CPU0センサーとI2C構成の設定値
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_SENSOR_H
#define SEROV_CPU0_CONFIG_SENSOR_H

#define CPU0_SENSOR_I2C_ENABLED            (1U)
#define CPU0_SENSOR_PERIOD_MS              (50U)
#define CPU0_SENSOR_RETRY_PERIOD_MS        (1000U)
#define CPU0_SENSOR_STALE_TIMEOUT_MS       (200U)
#define CPU0_SENSOR_I2C_TIMEOUT_MS         (20U)
#define CPU0_TOF_DATA_READY_TIMEOUT_MS     (100U)

/* TCA9548Aは常に対象一チャネルのみを有効にする。 */
#define CPU0_TCA9548A_ADDRESS              (0x70U)
#define CPU0_TCA9548A_CHANNEL_LEFT         (0U)
#define CPU0_TCA9548A_CHANNEL_CENTER       (1U)
#define CPU0_TCA9548A_CHANNEL_RIGHT        (2U)
#define CPU0_TCA9548A_CHANNEL_BMI270       (3U)
#define CPU0_VL53L1X_ADDRESS               (0x29U)
#define CPU0_TOF_MIN_VALID_MM              (40U)
#define CPU0_TOF_MAX_VALID_MM              (4000U)

#define CPU0_BMI270_RESET_DELAY_MS         (10U)
#define CPU0_BMI270_STARTUP_DELAY_MS       (50U)
#define CPU0_BMI270_ACCEL_LSB_PER_G        (8192)
#define CPU0_BMI270_GYRO_RANGE_DPS_X10     (5000)

#endif /* SEROV_CPU0_CONFIG_SENSOR_H */
