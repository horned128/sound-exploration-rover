/** =================================================================*
 * @file   safety_arbiter.c
 * @brief  CPU0安全調停実装（生存性・センサー有効性・出力クランプ）
 * ================================================================= */
#include "safety_arbiter.h"                                 /* 安全調停APIと指令型 */
#include "config/control_config.h"                          /* 制御上限値 */
#include "config/sensor_config.h"                           /* センサー安全閾値 */
#include <stddef.h>                                         /* NULL */

/** =================================================================*
 * @brief  符号付き16 bit値を範囲内へクランプ
 * @param[in] val 入力値
 * @param[in] min_val 下限
 * @param[in] max_val 上限
 * @return クランプ後の値
 * ================================================================= */
LOCAL H clamp_i16(H val, H min_val, H max_val) {
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

/** =================================================================*
 * @brief  符号付き32 bit値の絶対値を取得
 * @param[in] val 入力値
 * @return 絶対値
 * ================================================================= */
LOCAL W abs_i32(W val) {
    return (val < 0) ? -val : val;
}

/** =================================================================*
 * @brief  ToFスナップショットの前進利用可否を判定
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 3眼ToFが有効かつ期限内ならTRUE
 * ================================================================= */
EXPORT BOOL safety_arbiter_tof_usable(const sensor_snapshot_t * p_snapshot) {
    UB const required_tof_flags = (UB) (CPU0_SENSOR_VALID_TOF_LEFT |
                                        CPU0_SENSOR_VALID_TOF_CENTER |
                                        CPU0_SENSOR_VALID_TOF_RIGHT);
    if ((NULL == p_snapshot) || !p_snapshot->initialized ||
        (required_tof_flags != (p_snapshot->valid_flags & required_tof_flags)) ||
        (p_snapshot->age_ms > CPU0_SENSOR_STALE_TIMEOUT_MS)) {
        return FALSE;
    }

    return TRUE;
}

/** =================================================================*
 * @brief  IMU姿勢・衝撃の安全性を判定
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 姿勢・角速度・加速度が安全範囲ならTRUE
 * ================================================================= */
LOCAL BOOL safety_arbiter_imu_safe(const sensor_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return FALSE;
    }
    W acceleration_l1_mg = 0;
    for (UW axis = 0U; axis < 3U; axis++) {
        acceleration_l1_mg += abs_i32((W) p_snapshot->accel_mg[axis]);
        if (abs_i32((W) p_snapshot->gyro_dps_x10[axis]) > CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10) {
            return FALSE;
        }
    }
    return (abs_i32((W) p_snapshot->accel_mg[0]) <= CPU0_SENSOR_IMU_MAX_TILT_MG) &&
           (abs_i32((W) p_snapshot->accel_mg[1]) <= CPU0_SENSOR_IMU_MAX_TILT_MG) &&
           (acceleration_l1_mg <= CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG);
}

/** =================================================================*
 * @brief  センサー状態を集約して前進可否を判定
 * @param[in] p_snapshot 最新センサースナップショット
 * @param[in] sensor_fresh センサー更新の生存性
 * @param[out] p_status 判定内訳（NULL可）
 * @return 前進可能ならTRUE
 * ================================================================= */
EXPORT BOOL safety_arbiter_motion_allowed(const sensor_snapshot_t * p_snapshot,
                                          BOOL sensor_fresh,
                                          safety_arbiter_status_t * p_status) {
    BOOL const tof_ok = safety_arbiter_tof_usable(p_snapshot);
    BOOL const imu_ok = safety_arbiter_imu_safe(p_snapshot);
    BOOL const hard_stop = (NULL != p_snapshot) && p_snapshot->initialized &&
                           (!tof_ok) && (p_snapshot->age_ms <= CPU0_SENSOR_STALE_TIMEOUT_MS);
    BOOL const allowed = sensor_fresh && tof_ok && imu_ok;

    if (NULL != p_status) {
        p_status->sensor_fresh = sensor_fresh;
        p_status->tof_usable = tof_ok;
        p_status->motion_allowed = allowed;
        p_status->hard_stop_veto = hard_stop;
        p_status->imu_safe = imu_ok;
    }

    return allowed;
}

/** =================================================================*
 * @brief  走行要求へセンサー由来の安全制限を適用
 * @param[in] p_requested 上位制御の要求指令
 * @param[in] p_snapshot 最新センサースナップショット
 * @param[in] sensor_fresh センサー更新の生存性
 * @param[out] p_arbitrated 安全調停後の指令
 * ================================================================= */
EXPORT void safety_arbiter_arbitrate(const safety_motion_command_t * p_requested,
                                     const sensor_snapshot_t * p_snapshot,
                                     BOOL sensor_fresh,
                                     safety_motion_command_t * p_arbitrated) {
    if ((NULL == p_requested) || (NULL == p_arbitrated)) {
        return;
    }

    *p_arbitrated = *p_requested;

    /* 操舵角クランプ: [-45, +45] (I2) */
    p_arbitrated->steering_deg = clamp_i16(p_requested->steering_deg,
                                           -CPU0_SOUND_STEERING_MAX_DEG,
                                           CPU0_SOUND_STEERING_MAX_DEG);

    /* 生存性判定 (0-F / I7) */
    if (!sensor_fresh) {
        p_arbitrated->left_rpm = 0;
        p_arbitrated->right_rpm = 0;
        p_arbitrated->actuator_enable = FALSE;
        /* 通常のvetoでemergency_stopを立てない (A-3 / 4-4-3 / I6) */
        return;
    }

    /* IMU衝撃・転倒による安全停止 */
    if (!safety_arbiter_imu_safe(p_snapshot)) {
        p_arbitrated->left_rpm = 0;
        p_arbitrated->right_rpm = 0;
        p_arbitrated->actuator_enable = FALSE;
        return;
    }

    /* 前進要求時のToF有効性調停。距離閾値による即時停止は行わず、
     * 正面近接の連続確認と回避判断は obstacle_avoidance_controller が担う。 */
    BOOL const is_forward_request = (p_requested->left_rpm > 0) || (p_requested->right_rpm > 0);
    if (is_forward_request) {
        if (!safety_arbiter_tof_usable(p_snapshot)) {
            /* 無効・期限切れToFは前進を阻止する。 */
            p_arbitrated->left_rpm = 0;
            p_arbitrated->right_rpm = 0;
            p_arbitrated->actuator_enable = FALSE;
            /* 通常vetoでemergency_stopを立てない (A-3 / 4-4-3 / I6) */
        }
    }
}
