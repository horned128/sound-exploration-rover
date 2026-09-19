/** =================================================================*
 * @file   safety_arbiter.c
 * @brief  CPU0安全調停実装（生存性・ToF veto・出力クランプ）
 * ================================================================= */
#include "safety_arbiter.h"
#include "config/control_config.h"
#include "config/sensor_config.h"
#include <stddef.h>

static int16_t clamp_i16(int16_t val, int16_t min_val, int16_t max_val) {
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

static int32_t abs_i32(int32_t val) {
    return (val < 0) ? -val : val;
}

bool safety_arbiter_tof_usable(const sensor_snapshot_t * p_snapshot) {
    uint8_t const required_tof_flags = (uint8_t) (CPU0_SENSOR_VALID_TOF_LEFT |
                                                  CPU0_SENSOR_VALID_TOF_CENTER |
                                                  CPU0_SENSOR_VALID_TOF_RIGHT);
    if ((NULL == p_snapshot) || !p_snapshot->initialized ||
        (required_tof_flags != (p_snapshot->valid_flags & required_tof_flags)) ||
        (p_snapshot->age_ms > CPU0_SENSOR_STALE_TIMEOUT_MS)) {
        return false;
    }

    for (uint32_t index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        if (p_snapshot->tof_distance_mm[index] < CPU0_SENSOR_HARD_STOP_DISTANCE_MM) {
            return false;
        }
    }
    return true;
}

static bool safety_arbiter_imu_safe(const sensor_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return false;
    }
    int32_t acceleration_l1_mg = 0;
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        acceleration_l1_mg += abs_i32((int32_t) p_snapshot->accel_mg[axis]);
        if (abs_i32((int32_t) p_snapshot->gyro_dps_x10[axis]) > CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10) {
            return false;
        }
    }
    return (abs_i32((int32_t) p_snapshot->accel_mg[0]) <= CPU0_SENSOR_IMU_MAX_TILT_MG) &&
           (abs_i32((int32_t) p_snapshot->accel_mg[1]) <= CPU0_SENSOR_IMU_MAX_TILT_MG) &&
           (acceleration_l1_mg <= CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG);
}

bool safety_arbiter_motion_allowed(const sensor_snapshot_t * p_snapshot,
                                   bool sensor_fresh,
                                   safety_arbiter_status_t * p_status) {
    bool const tof_ok = safety_arbiter_tof_usable(p_snapshot);
    bool const imu_ok = safety_arbiter_imu_safe(p_snapshot);
    bool const hard_stop = (NULL != p_snapshot) && p_snapshot->initialized &&
                           (!tof_ok) && (p_snapshot->age_ms <= CPU0_SENSOR_STALE_TIMEOUT_MS);
    bool const allowed = sensor_fresh && tof_ok && imu_ok;

    if (NULL != p_status) {
        p_status->sensor_fresh = sensor_fresh;
        p_status->tof_usable = tof_ok;
        p_status->motion_allowed = allowed;
        p_status->hard_stop_veto = hard_stop;
        p_status->imu_safe = imu_ok;
    }

    return allowed;
}

void safety_arbiter_arbitrate(const safety_motion_command_t * p_requested,
                             const sensor_snapshot_t * p_snapshot,
                             bool sensor_fresh,
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
        p_arbitrated->actuator_enable = false;
        /* 通常のvetoでemergency_stopを立てない (A-3 / 4-4-3 / I6) */
        return;
    }

    /* IMU衝撃・転倒による安全停止 */
    if (!safety_arbiter_imu_safe(p_snapshot)) {
        p_arbitrated->left_rpm = 0;
        p_arbitrated->right_rpm = 0;
        p_arbitrated->actuator_enable = false;
        return;
    }

    /* 前進要求時のToF安全調停 (I1) */
    bool const is_forward_request = (p_requested->left_rpm > 0) || (p_requested->right_rpm > 0);
    if (is_forward_request) {
        if (!safety_arbiter_tof_usable(p_snapshot)) {
            /* ハード停止: 前進を阻止し停止 (I1) */
            p_arbitrated->left_rpm = 0;
            p_arbitrated->right_rpm = 0;
            p_arbitrated->actuator_enable = false;
            /* 通常vetoでemergency_stopを立てない (A-3 / 4-4-3 / I6) */
        }
    }
}
