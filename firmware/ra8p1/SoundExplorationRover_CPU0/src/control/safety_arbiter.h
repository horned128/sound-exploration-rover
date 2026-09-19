/** =================================================================*
 * @file   safety_arbiter.h
 * @brief  CPU0安全調停API（生存性・ToF veto・出力クランプ）
 * ================================================================= */
#ifndef SEROV_CPU0_SAFETY_ARBITER_H
#define SEROV_CPU0_SAFETY_ARBITER_H

#include "services/sensor_hub.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct st_safety_motion_command {
    int16_t steering_deg;           /**< 要求操舵角 [deg] */
    int16_t left_rpm;               /**< 左車輪要求RPM */
    int16_t right_rpm;              /**< 右車輪要求RPM */
    bool    actuator_enable;        /**< 出力許可 */
    bool    emergency_stop;         /**< 非常停止 */
} safety_motion_command_t;

typedef struct st_safety_arbiter_status {
    bool sensor_fresh;              /**< sensor_liveness判定結果 */
    bool tof_usable;                /**< 3眼ToFマスク充足かつハード停止距離以上 */
    bool motion_allowed;            /**< 前進走行許可フラグ (fresh && tof_usable) */
    bool hard_stop_veto;            /**< 250mm未満のToFによるveto発火 */
    bool imu_safe;                  /**< IMU姿勢・衝撃の安全判定 */
} safety_arbiter_status_t;

/** =================================================================*
 * @brief  ToFスナップショットが前進可能か判定（マスク意味論）
 * @details IMUには依存せず、左右・中央の3眼ToFのみをマスク検査する。
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 3眼ToFが有効かつ250mm以上ならtrue
 * ================================================================= */
bool safety_arbiter_tof_usable(const sensor_snapshot_t * p_snapshot);

/** =================================================================*
 * @brief  総合前進可否判定（生存性＋ToF）
 * @param[in]  p_snapshot   最新センサースナップショット
 * @param[in]  sensor_fresh 生存性判定結果 (sensor_liveness)
 * @param[out] p_status     状態詳細出力（NULL可）
 * @return 前進許可ならtrue
 * ================================================================= */
bool safety_arbiter_motion_allowed(const sensor_snapshot_t * p_snapshot,
                                   bool sensor_fresh,
                                   safety_arbiter_status_t * p_status);

/** =================================================================*
 * @brief  走行指令の安全調停・クランプ
 * @param[in]  p_requested  要求走行指令
 * @param[in]  p_snapshot   最新センサースナップショット
 * @param[in]  sensor_fresh 生存性判定結果
 * @param[out] p_arbitrated 調停後走行指令
 * ================================================================= */
void safety_arbiter_arbitrate(const safety_motion_command_t * p_requested,
                             const sensor_snapshot_t * p_snapshot,
                             bool sensor_fresh,
                             safety_motion_command_t * p_arbitrated);

#ifdef __cplusplus
}
#endif

#endif /* SEROV_CPU0_SAFETY_ARBITER_H */
