/** =================================================================*
 * @file   tk_sensor.h
 * @brief  CPU0センサー取得タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TK_SENSOR_H
#define SEROV_CPU0_TK_SENSOR_H

#include "../sensors/sensor_hub.h"                          /* センサースナップショット型 */
#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

EXPORT cpu0_fault_t cpu0_sensor_task_create(void);          /* センサータスクとmutex生成 */
EXPORT cpu0_fault_t cpu0_sensor_task_start(void);           /* センサータスク開始 */
EXPORT void cpu0_sensor_task_delete(void);                  /* センサータスクとmutex解放 */
EXPORT ER cpu0_sensor_snapshot_get(cpu0_sensor_snapshot_t * p_snapshot); /* 最新値取得 */

IMPORT volatile UH g_cpu0_sensor_tof_distance_mm[CPU0_SENSOR_TOF_COUNT]; /* ToF距離[mm] */
IMPORT volatile H g_cpu0_sensor_accel_mg[3];                 /* IMU加速度[mg] */
IMPORT volatile H g_cpu0_sensor_gyro_dps_x10[3];             /* IMU角速度[0.1dps] */
IMPORT volatile UW g_cpu0_sensor_age_ms;                     /* 最新値の経過時間 */
IMPORT volatile UW g_cpu0_sensor_update_count;               /* 正常更新回数 */
IMPORT volatile UW g_cpu0_sensor_error_flags;                /* 最終センサーerror bit */
IMPORT volatile W g_cpu0_sensor_last_error;                  /* 最終FSPエラー */
IMPORT volatile UB g_cpu0_sensor_valid_flags;                /* ToF/IMU valid bit */
IMPORT volatile BOOL g_cpu0_sensor_initialized;              /* 全センサー初期化状態 */
IMPORT volatile UB g_cpu0_sensor_tof_range_status[CPU0_SENSOR_TOF_COUNT]; /* ToF raw Range Status */
IMPORT volatile UB g_cpu0_sensor_tof_result[CPU0_SENSOR_TOF_COUNT]; /* ToF読出し結果分類 */
IMPORT volatile UB g_cpu0_sensor_failure_kind;               /* 最終失敗分類 */
IMPORT volatile UB g_cpu0_sensor_failure_device;             /* 最終失敗device */
IMPORT volatile UB g_cpu0_sensor_failure_stage;              /* 最終失敗stage */
IMPORT volatile B g_cpu0_sensor_failure_channel;             /* 最終失敗TCAチャネル */
IMPORT volatile UW g_cpu0_sensor_i2c_transfer_timeout_count; /* I2C転送timeout累積回数 */
IMPORT volatile UW g_cpu0_sensor_tof_data_ready_timeout_count[CPU0_SENSOR_TOF_COUNT]; /* ToF ready timeout回数 */
IMPORT volatile UW g_cpu0_sensor_invalid_data_count[CPU0_SENSOR_TOF_COUNT]; /* ToF測距無効回数 */
IMPORT volatile UW g_cpu0_sensor_hub_recovery_count;         /* I2C障害後hub再初期化回数 */

#endif /* SEROV_CPU0_TK_SENSOR_H */
