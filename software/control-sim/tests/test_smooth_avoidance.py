"""Tests for the smooth obstacle avoidance planner (potential-field pure function)."""

from __future__ import annotations

import math
import pytest
from controlsim.bindings import (
    SmoothAvoidanceInput,
    SmoothAvoidanceOutput,
    smooth_avoidance_plan_step,
)


def make_input(
    tof_mm: tuple[float, float, float] = (1500.0, 1500.0, 1500.0),
    tof_valid: tuple[bool, bool, bool] = (True, True, True),
    target_heading_deg: float = 0.0,
    current_steering_deg: float = 0.0,
    current_speed_scale: float = 1.0,
    dt_sec: float = 0.05,
) -> SmoothAvoidanceInput:
    inp = SmoothAvoidanceInput()
    inp.tof_distance_mm[0] = tof_mm[0]
    inp.tof_distance_mm[1] = tof_mm[1]
    inp.tof_distance_mm[2] = tof_mm[2]
    inp.tof_valid[0] = tof_valid[0]
    inp.tof_valid[1] = tof_valid[1]
    inp.tof_valid[2] = tof_valid[2]
    inp.target_heading_deg = target_heading_deg
    inp.current_steering_deg = current_steering_deg
    inp.current_speed_scale = current_speed_scale
    inp.dt_sec = dt_sec
    return inp


# --- 不変条件テスト (Invariants) ---

def test_i1_close_distance_does_not_stop_by_itself():
    """距離だけでは停止せず、近接しても最小走行速度を維持する。"""
    for idx in range(3):
        tof = [1000.0, 1000.0, 1000.0]
        tof[idx] = 249.0
        inp = make_input(tof_mm=tuple(tof))
        out = smooth_avoidance_plan_step(inp)
        assert not out.is_blocked, f"ToF channel {idx} close reading must not stop by distance"
        assert out.speed_scale > 0.0, f"ToF channel {idx} must keep moving"


def test_i2_steering_clamp():
    """I2: 操舵角は常に [-45, +45] 度以内にクランプされる。"""
    headings = [-180.0, -120.0, -90.0, -45.0, 0.0, 45.0, 90.0, 120.0, 180.0]
    for h in headings:
        inp = make_input(target_heading_deg=h, current_steering_deg=0.0, dt_sec=1.0)
        out = smooth_avoidance_plan_step(inp)
        assert -45.0 <= out.steering_deg <= 45.0


def test_i3_steering_rate_limit():
    """I3: 連続呼出し間の操舵角変化は角速度制限以内 (90 deg/s)。"""
    dt = 0.05
    max_step = 90.0 * dt + 1e-4
    inp = make_input(
        target_heading_deg=45.0,
        current_steering_deg=0.0,
        dt_sec=dt,
    )
    out = smooth_avoidance_plan_step(inp)
    assert abs(out.steering_deg - inp.current_steering_deg) <= max_step


def test_i4_speed_rate_limit():
    """I4: 連続呼出し間の速度スケール変化は加速度制限以内 (1.5 /s)。"""
    dt = 0.05
    max_step = 1.5 * dt + 1e-4
    # 遠方クリアランスで速度1.0を目指すが現在0.2
    inp = make_input(
        tof_mm=(2000.0, 2000.0, 2000.0),
        current_speed_scale=0.2,
        dt_sec=dt,
    )
    out = smooth_avoidance_plan_step(inp)
    assert abs(out.speed_scale - inp.current_speed_scale) <= max_step


def test_i5_invalid_channel_distance_ignored():
    """I5: valid_flag=False のチャネルの距離値 (0mmなど) は近接停止や回避に影響しない。"""
    # 左ToFが無効で距離0mmが入っている場合
    inp = make_input(
        tof_mm=(0.0, 1000.0, 1000.0),
        tof_valid=(False, True, True),
        target_heading_deg=0.0,
    )
    out = smooth_avoidance_plan_step(inp)
    assert not out.is_blocked
    assert out.speed_scale > 0.0


def test_i6_all_invalid_stops():
    """I6: 全ToFが無効の場合、安全のため停止する。"""
    inp = make_input(
        tof_mm=(0.0, 0.0, 0.0),
        tof_valid=(False, False, False),
    )
    out = smooth_avoidance_plan_step(inp)
    assert out.is_blocked
    assert out.speed_scale == 0.0


def test_i9_deadzone_avoidance_when_driving():
    """I9: 停止意図でない走行時、速度スケールは 0.20 を下回らない。"""
    # 障害物が接近していても、停止意図がなければ最低速度を維持する。
    inp = make_input(
        tof_mm=(300.0, 300.0, 300.0),
        current_speed_scale=0.5,
        dt_sec=1.0,
    )
    out = smooth_avoidance_plan_step(inp)
    assert not out.is_blocked
    assert out.speed_scale >= 0.20


# --- 合成シナリオテスト (Scenarios) ---

def test_s1_head_on_wall_approach_and_stop():
    """S1: 正面の壁に接近すると徐々に減速し、近接しても停止はしない。"""
    speeds = []
    # 1000mmから200mmまで接近
    for dist in [1000.0, 800.0, 600.0, 450.0, 300.0, 250.0, 240.0]:
        inp = make_input(
            tof_mm=(dist, dist, dist),
            current_speed_scale=1.0,
            dt_sec=1.0,  # 減速応答を直接確認するためdtを十分大きく
        )
        out = smooth_avoidance_plan_step(inp)
        speeds.append(out.speed_scale)

    # 1000mm時は1.0、600mm時は減速し、240mmでも走行スケールを残す
    assert speeds[0] == 1.0
    assert speeds[2] < speeds[0]
    assert speeds[-1] > 0.0


def test_s2_corridor_centering():
    """S2: 廊下（左右対称の壁）では直進を維持する。"""
    inp = make_input(
        tof_mm=(400.0, 1500.0, 400.0),
        target_heading_deg=0.0,
        current_steering_deg=0.0,
        dt_sec=0.1,
    )
    out = smooth_avoidance_plan_step(inp)
    assert abs(out.steering_deg) < 1.0, "Symmetric corridor should keep steering near zero"


def test_s3_obstacle_on_one_side_steers_away():
    """S3: 片側に障害物がある場合、反対側へ滑らかに曲がる。"""
    # 左に障害物 (L: 350mm, C: 1500mm, R: 1500mm) → 右 (正の舵角) へ回避
    inp_left_obs = make_input(
        tof_mm=(350.0, 1500.0, 1500.0),
        target_heading_deg=0.0,
        current_steering_deg=0.0,
        dt_sec=0.5,
    )
    out_left = smooth_avoidance_plan_step(inp_left_obs)
    assert out_left.steering_deg > 5.0, "Left obstacle must steer right (positive)"

    # 右に障害物 (L: 1500mm, C: 1500mm, R: 350mm) → 左 (負の舵角) へ回避
    inp_right_obs = make_input(
        tof_mm=(1500.0, 1500.0, 350.0),
        target_heading_deg=0.0,
        current_steering_deg=0.0,
        dt_sec=0.5,
    )
    out_right = smooth_avoidance_plan_step(inp_right_obs)
    assert out_right.steering_deg < -5.0, "Right obstacle must steer left (negative)"


def test_s5_single_tof_failure_continues_operation():
    """S5: 1本のToFが無効になっても、残り2本で走行を継続できる。"""
    inp = make_input(
        tof_mm=(1500.0, 1500.0, 0.0),
        tof_valid=(True, True, False),
        target_heading_deg=0.0,
    )
    out = smooth_avoidance_plan_step(inp)
    assert not out.is_blocked
    assert out.speed_scale > 0.0


def test_s8_rear_target_turns_forward():
    """S8: 目標方位が後方 (+150度) でも、その場旋回せず前進しながら最大舵角で曲がる。"""
    inp = make_input(
        tof_mm=(1500.0, 1500.0, 1500.0),
        target_heading_deg=150.0,
        current_steering_deg=0.0,
        current_speed_scale=0.5,
        dt_sec=1.0,
    )
    out = smooth_avoidance_plan_step(inp)
    assert not out.is_blocked
    assert out.speed_scale > 0.0, "Must continue moving forward"
    assert out.steering_deg == pytest.approx(45.0, abs=1.0), "Must steer sharply towards target"
