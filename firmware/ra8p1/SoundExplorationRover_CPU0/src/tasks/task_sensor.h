/** =================================================================*
 * @file   task_sensor.h
 * @brief  CPU0センサー取得タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_SENSOR_H
#define SEROV_CPU0_TASK_SENSOR_H

#include "services/sensor_hub.h"                            /* センサースナップショット型 */
#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

EXPORT app_fault_t task_sensor_create(void);                /* センサータスクとmutex生成 */
EXPORT app_fault_t task_sensor_start(void);                 /* センサータスク開始 */
EXPORT void task_sensor_delete(void);                       /* センサータスクとmutex解放 */
EXPORT ER task_sensor_snapshot_get(sensor_snapshot_t * p_snapshot); /* 最新スナップショット取得 */

/**< ToF距離[mm] */
IMPORT volatile UH g_task_sensor_tof_distance_mm[CPU0_SENSOR_TOF_COUNT];
IMPORT volatile H g_task_sensor_accel_mg[3];                /**< IMU加速度[mg] */
IMPORT volatile H g_task_sensor_gyro_dps_x10[3];            /**< IMU角速度[0.1dps] */
IMPORT volatile UW g_task_sensor_age_ms;                    /**< 最新値の経過時間[ms] */
IMPORT volatile UW g_task_sensor_update_count;              /**< 正常更新回数 */
IMPORT volatile UW g_task_sensor_error_flags;               /**< 現在のセンサー異常ビット */
IMPORT volatile W g_task_sensor_last_error;                 /**< 現在のFSPエラー */
IMPORT volatile UB g_task_sensor_valid_flags;               /**< ToF/IMU有効ビット */
IMPORT volatile BOOL g_task_sensor_initialized;             /**< 全センサー初期化状態 */
/**< ToF生Range Status */
IMPORT volatile UB g_task_sensor_tof_range_status[CPU0_SENSOR_TOF_COUNT];
/**< ToF読出し結果分類 */
IMPORT volatile UB g_task_sensor_tof_result[CPU0_SENSOR_TOF_COUNT];
IMPORT volatile UB g_task_sensor_failure_kind;              /**< 現在の失敗分類 */
IMPORT volatile UB g_task_sensor_failure_device;            /**< 現在の失敗デバイス */
IMPORT volatile UB g_task_sensor_failure_stage;             /**< 現在の失敗段階 */
IMPORT volatile B g_task_sensor_failure_channel;            /**< 現在の失敗TCAチャネル */
IMPORT volatile UW g_task_sensor_i2c_transfer_timeout_count;/**< I2C転送タイムアウト累積回数 */
/**< ToF準備timeout回数 */
IMPORT volatile UW g_task_sensor_tof_data_ready_timeout_count[CPU0_SENSOR_TOF_COUNT];
/**< ToF測距無効回数 */
IMPORT volatile UW g_task_sensor_invalid_data_count[CPU0_SENSOR_TOF_COUNT];
IMPORT volatile UW g_task_sensor_hub_recovery_count;        /**< I2C障害後ハブ再初期化回数 */

IMPORT volatile BOOL g_task_sensor_test_pause;              /**< 更新停止試験用、既定FALSE */

IMPORT volatile W  g_task_sensor_odometry_x_mm;             /**< オドメトリ推定X[mm] */
IMPORT volatile W  g_task_sensor_odometry_y_mm;             /**< オドメトリ推定Y[mm] */
IMPORT volatile H  g_task_sensor_odometry_theta_deg_x10;    /**< オドメトリ推定方位[0.1 deg] */
IMPORT volatile UW g_task_sensor_odometry_distance_mm;      /**< オドメトリ累積走行距離[mm] */
IMPORT volatile BOOL g_task_sensor_odometry_valid;          /**< オドメトリ有効フラグ */

#endif /* SEROV_CPU0_TASK_SENSOR_H */
