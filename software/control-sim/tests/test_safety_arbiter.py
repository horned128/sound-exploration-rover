"""Tests for safety_arbiter (Phase 4-5 S1: ToF veto and liveness aggregation)."""

from __future__ import annotations

import pytest
from controlsim.bindings import (
    SafetyMotionCommand,
    SafetyArbiterStatus,
    SensorSnapshot,
    safety_arbiter_arbitrate_step,
    safety_arbiter_check_allowed,
)

# Mask constants
CPU0_SENSOR_VALID_TOF_LEFT = 1 << 0
CPU0_SENSOR_VALID_TOF_CENTER = 1 << 1
CPU0_SENSOR_VALID_TOF_RIGHT = 1 << 2
CPU0_SENSOR_VALID_IMU = 1 << 3
CPU0_SENSOR_VALID_TOF_ALL = (
    CPU0_SENSOR_VALID_TOF_LEFT | CPU0_SENSOR_VALID_TOF_CENTER | CPU0_SENSOR_VALID_TOF_RIGHT
)


def make_snapshot(
    tof_mm: tuple[int, int, int] = (1000, 1000, 1000),
    valid_flags: int = CPU0_SENSOR_VALID_TOF_ALL | CPU0_SENSOR_VALID_IMU,
    age_ms: int = 10,
    initialized: bool = True,
) -> SensorSnapshot:
    snap = SensorSnapshot()
    snap.initialized = initialized
    snap.valid_flags = valid_flags
    snap.age_ms = age_ms
    snap.tof_distance_mm[0] = tof_mm[0]
    snap.tof_distance_mm[1] = tof_mm[1]
    snap.tof_distance_mm[2] = tof_mm[2]
    return snap


def make_command(
    steering_deg: int = 0,
    left_rpm: int = 100,
    right_rpm: int = 100,
    actuator_enable: bool = True,
    emergency_stop: bool = False,
) -> SafetyMotionCommand:
    cmd = SafetyMotionCommand()
    cmd.steering_deg = steering_deg
    cmd.left_rpm = left_rpm
    cmd.right_rpm = right_rpm
    cmd.actuator_enable = actuator_enable
    cmd.emergency_stop = emergency_stop
    return cmd


def test_tof_mask_semantics_independent_of_imu():
    """S1要件: マスク意味論（ToF 3眼のみ要求し、IMU valid_flag に依存しない）。"""
    # IMUフラグがない (VALID_TOF_ALL のみ)
    snap = make_snapshot(valid_flags=CPU0_SENSOR_VALID_TOF_ALL)
    allowed, status = safety_arbiter_check_allowed(snap, sensor_fresh=True)
    assert allowed
    assert status.tof_usable
    assert not status.hard_stop_veto


def test_close_tof_does_not_veto_forward_motion():
    """距離だけでは停止せず、正面の連続衝突判定を回避制御へ委譲する。"""
    for idx in range(3):
        tof = [1000, 1000, 1000]
        tof[idx] = 240
        snap = make_snapshot(tof_mm=tuple(tof))
        cmd = make_command(left_rpm=120, right_rpm=120)
        arb = safety_arbiter_arbitrate_step(cmd, snap, sensor_fresh=True)

        assert arb.actuator_enable, f"ToF channel {idx} close reading must not veto by distance"
        assert arb.left_rpm == 120
        assert arb.right_rpm == 120
        assert not arb.emergency_stop


def test_hard_stop_veto_allows_reverse():
    """前方ToFのvetoは後退（バック）を阻止しない。"""
    snap = make_snapshot(tof_mm=(200, 200, 200))  # 前方障害物
    cmd = make_command(left_rpm=-100, right_rpm=-100)  # 後退
    arb = safety_arbiter_arbitrate_step(cmd, snap, sensor_fresh=True)

    assert arb.actuator_enable
    assert arb.left_rpm == -100
    assert arb.right_rpm == -100


def test_sensor_fresh_false_stops_vehicle():
    """0-F / I7要件: センサー更新が停止 (sensor_fresh=False) したら停止側へ倒れる。"""
    snap = make_snapshot(tof_mm=(1000, 1000, 1000))
    cmd = make_command(left_rpm=100, right_rpm=100)
    arb = safety_arbiter_arbitrate_step(cmd, snap, sensor_fresh=False)

    assert not arb.actuator_enable
    assert arb.left_rpm == 0
    assert arb.right_rpm == 0
    assert not arb.emergency_stop


def test_steering_clamp():
    """I2要件: 操舵角は [-45, +45] にクランプされる。"""
    snap = make_snapshot()
    cmd = make_command(steering_deg=60)
    arb = safety_arbiter_arbitrate_step(cmd, snap, sensor_fresh=True)
    assert arb.steering_deg == 45

    cmd2 = make_command(steering_deg=-70)
    arb2 = safety_arbiter_arbitrate_step(cmd2, snap, sensor_fresh=True)
    assert arb2.steering_deg == -45
