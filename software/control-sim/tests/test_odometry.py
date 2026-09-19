"""Tests for the odometry service and complementary filter."""

from __future__ import annotations

import ctypes
import math
import pytest

from controlsim.bindings import OdometryContext, OdometryPose, library


@pytest.fixture
def odom():
    lib = library()
    ctx = OdometryContext()
    lib.odometry_init(ctypes.byref(ctx))
    return lib, ctx


def get_pose(lib, ctx) -> OdometryPose:
    pose = OdometryPose()
    lib.odometry_get_pose(ctypes.byref(ctx), ctypes.byref(pose))
    return pose


def test_odometry_initialization(odom):
    lib, ctx = odom
    pose = get_pose(lib, ctx)
    assert not pose.valid
    assert pose.x_mm == 0
    assert pose.y_mm == 0
    assert pose.theta_deg_x10 == 0

    # First update synchronizes base counts without moving
    lib.odometry_update(ctypes.byref(ctx), 1000, 2000, 100, 0, 0, 0)
    pose = get_pose(lib, ctx)
    assert pose.valid
    assert pose.x_mm == 0
    assert pose.y_mm == 0
    assert pose.total_distance_mm == 0
    assert pose.left_encoder_total == 0
    assert pose.right_encoder_total == 0


def test_odometry_straight_forward_motion(odom):
    lib, ctx = odom
    # Initial sync at t=0 ms
    lib.odometry_update(ctypes.byref(ctx), 0, 0, 0, 0, 0, 0)

    # 1 revolution forward on both wheels = 702 counts = 339.29 mm
    # t = 1000 ms (1 sec), RPM = 600 (60.0 RPM)
    lib.odometry_update(ctypes.byref(ctx), 702, 702, 1000, 0, 600, 600)
    pose = get_pose(lib, ctx)

    assert pose.valid
    assert pytest.approx(pose.total_distance_mm, abs=2) == 339
    # Heading is 0 rad (forward along X axis: dx = d * cos(0) = 339 mm)
    assert pytest.approx(pose.x_mm, abs=2) == 339
    assert pytest.approx(pose.y_mm, abs=2) == 0
    assert pose.theta_deg_x10 == 0
    assert pytest.approx(pose.linear_speed_mm_s, abs=2) == 339
    assert pose.left_encoder_total == 702
    assert pose.right_encoder_total == 702


def test_odometry_24bit_wrap_forward(odom):
    lib, ctx = odom
    # Sync near 24-bit boundary (0x00FFFFF0)
    start_count = 0x00FFFFF0
    lib.odometry_update(ctypes.byref(ctx), start_count, start_count, 1000, 0, 0, 0)

    # Wrap across 0x00FFFFFF to 0x00000010 (+32 counts)
    end_count = 0x00000010
    lib.odometry_update(ctypes.byref(ctx), end_count, end_count, 1100, 0, 100, 100)
    pose = get_pose(lib, ctx)

    assert pose.left_encoder_total == 32
    assert pose.right_encoder_total == 32
    expected_dist_mm = 32 * 0.48332194
    assert pytest.approx(pose.total_distance_mm, abs=1) == round(expected_dist_mm)


def test_odometry_24bit_wrap_backward(odom):
    lib, ctx = odom
    # Sync near 0
    lib.odometry_update(ctypes.byref(ctx), 10, 10, 1000, 0, 0, 0)

    # Reverse across 0 to 0x00FFFFF6 (-20 counts)
    end_count = 0x00FFFFF6
    lib.odometry_update(ctypes.byref(ctx), end_count, end_count, 1100, 0, -100, -100)
    pose = get_pose(lib, ctx)

    assert pose.left_encoder_total == -20
    assert pose.right_encoder_total == -20
    expected_dist_mm = 20 * 0.48332194
    assert pytest.approx(pose.total_distance_mm, abs=1) == round(expected_dist_mm)


def test_odometry_pivot_turn_wheel_differential(odom):
    lib, ctx = odom
    lib.odometry_update(ctypes.byref(ctx), 0, 0, 0, 0, 0, 0)

    # Pivot turn: Left wheel backward, Right wheel forward
    # Track width = 260 mm. 90 deg turn (pi/2 rad):
    # Arc per wheel = (pi / 2) * (260 / 2) = 204.2 mm
    # Counts = 204.2 / 0.48332 = 422.5 counts
    left_count = (-423) & 0x00FFFFFF
    right_count = 423
    # 90 deg in 1000 ms = 90 deg/s = 900 in 0.1 dps
    lib.odometry_update(ctypes.byref(ctx), left_count, right_count, 1000, 900, -300, 300)
    pose = get_pose(lib, ctx)

    # Pivot turn has zero net translation at vehicle center
    assert pytest.approx(pose.x_mm, abs=5) == 0
    assert pytest.approx(pose.y_mm, abs=5) == 0
    # Heading should be approximately +90 deg (+900 in 0.1 deg)
    assert pytest.approx(pose.theta_deg_x10, abs=30) == 900


def test_odometry_gyro_stationary_bias_calibration(odom):
    lib, ctx = odom
    lib.odometry_update(ctypes.byref(ctx), 100, 100, 0, 0, 0, 0)

    # Rover is stationary, but gyro has a +1.5 dps bias (15 in 0.1 dps)
    for t in range(50, 1050, 50):
        lib.odometry_update(ctypes.byref(ctx), 100, 100, t, 15, 0, 0)

    # After stationary updates, gyro bias should converge toward 1.5 dps
    assert pytest.approx(ctx.gyro_bias_dps, abs=0.2) == 1.5
    # Heading should remain 0 because vehicle is detected as stationary
    pose = get_pose(lib, ctx)
    assert pose.theta_deg_x10 == 0
