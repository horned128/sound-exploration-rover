/** =================================================================*
 * @file   safety_arbiter.h
 * @brief  CPU0安全調停API（生存性・センサー有効性・出力クランプ）
 * ================================================================= */
#ifndef SEROV_CPU0_SAFETY_ARBITER_H
#define SEROV_CPU0_SAFETY_ARBITER_H

#include "services/sensor_hub.h"                            /* センサースナップショット型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型 */

#ifdef __cplusplus
extern "C" {
#endif

/**< 安全調停へ入力・出力する走行指令 */
typedef struct st_safety_motion_command {
    H steering_deg;                                         /**< 要求操舵角 [deg] */
    H left_rpm;                                             /**< 左車輪要求RPM */
    H right_rpm;                                            /**< 右車輪要求RPM */
    BOOL actuator_enable;                                   /**< 出力許可 */
    BOOL emergency_stop;                                    /**< 非常停止 */
} safety_motion_command_t;

/**< 生存性・ToF・IMUを統合した安全判定状態 */
typedef struct st_safety_arbiter_status {
    BOOL sensor_fresh;                                      /**< sensor_liveness判定結果 */
    BOOL tof_usable;                                        /**< ToF有効かつ期限内 */
    BOOL motion_allowed;                                    /**< 前進走行許可フラグ (fresh && tof_usable) */
    BOOL hard_stop_veto;                                    /**< 無効・期限切れセンサーによる停止発火 */
    BOOL imu_safe;                                          /**< IMU姿勢・衝撃の安全判定 */
} safety_arbiter_status_t;

EXPORT BOOL safety_arbiter_tof_usable(const sensor_snapshot_t * p_snapshot); /* ToF安全判定 */

EXPORT BOOL safety_arbiter_motion_allowed(const sensor_snapshot_t * p_snapshot,
                                          BOOL sensor_fresh,
                                          safety_arbiter_status_t * p_status); /* 前進可否判定 */

EXPORT void safety_arbiter_arbitrate(const safety_motion_command_t * p_requested,
                                     const sensor_snapshot_t * p_snapshot,
                                     BOOL sensor_fresh,
                                     safety_motion_command_t * p_arbitrated); /* 指令安全調停 */

#ifdef __cplusplus
}
#endif

#endif /* SEROV_CPU0_SAFETY_ARBITER_H */
