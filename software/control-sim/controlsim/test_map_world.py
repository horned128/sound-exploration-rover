"""Simulation world for tmp/test_map.drawio test environment.

Test environment layout:
- Room size: 3100 mm x 3100 mm (-450 <= x <= 2650, -1550 <= y <= 1550)
- Rover start: (x=0, y=0), heading=0 deg (looking +X)
- Cylindrical obstacle: r = 110 mm (radius 11cm), placed 900 mm in front of rover
  Center at (x = 150 + 900 + 110 = 1160 mm, y = 0 mm)
- Continuous sound source: (x = 2200 mm, y = 0 mm)
- Side ToFs tilted outward by side_angle_deg (default 13.0 deg)
"""

from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass
from typing import Callable

from .bindings import (
    ACOUSTIC_XVF_STATUS_READY,
    AcousticObservation,
    ObstacleAvoidanceOutput,
    SensorSnapshot,
    SoundFollowInput,
    SoundFollowOutput,
    library,
)
from .scenarios import sensor_snapshot


@dataclass(frozen=True)
class RoverTrajectorySample:
    time_ms: int
    x_mm: float
    y_mm: float
    heading_deg: float
    steering_deg: float
    left_rpm: float
    right_rpm: float
    is_spin_turn: bool
    obstacle_clearance_mm: float
    wall_clearance_mm: float
    distance_to_sound_mm: float
    doa_relative_deg: float
    state: int
    rule: int


def intersect_ray_circle(
    ox: float, oy: float, vx: float, vy: float, cx: float, cy: float, r: float
) -> float:
    """Return distance from ray origin along (vx, vy) to circle (cx, cy, r)."""
    dx = ox - cx
    dy = oy - cy
    # A = vx*vx + vy*vy == 1.0 (assuming normalized vector)
    b = 2.0 * (dx * vx + dy * vy)
    c = dx * dx + dy * dy - r * r
    disc = b * b - 4.0 * c
    if disc < 0:
        return 4000.0
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / 2.0
    t2 = (-b + sqrt_disc) / 2.0
    if t1 > 0:
        return t1
    if t2 > 0:
        return t2
    return 4000.0


def intersect_ray_room(
    ox: float, oy: float, vx: float, vy: float,
    x_min: float = -450.0, x_max: float = 2650.0,
    y_min: float = -1550.0, y_max: float = 1550.0,
) -> float:
    """Return distance to closest room bounding wall along ray."""
    dists = []
    if vx > 1e-6:
        dists.append((x_max - ox) / vx)
    elif vx < -1e-6:
        dists.append((x_min - ox) / vx)
    if vy > 1e-6:
        dists.append((y_max - oy) / vy)
    elif vy < -1e-6:
        dists.append((y_min - oy) / vy)
    pos_dists = [d for d in dists if d > 0]
    return min(pos_dists) if pos_dists else 4000.0


def run_test_map_simulation(
    *,
    speed_mm_s: float = 240.0,
    turn_rate_dps: float = 25.0,
    initial_x_mm: float = 0.0,
    initial_y_mm: float = 0.0,
    initial_heading_deg: float = 0.0,
    duration_ms: int = 15000,
    side_angle_deg: float = 13.0,
    obstacle_x_mm: float = 1160.0,
    obstacle_y_mm: float = 0.0,
    obstacle_radius_mm: float = 110.0,
    sound_x_mm: float = 2200.0,
    sound_y_mm: float = 0.0,
) -> list[RoverTrajectorySample]:
    """Run closed-loop simulation of the full control pipeline on tmp/test_map.drawio."""
    handle = library()
    handle.sound_follow_controller_init()
    handle.obstacle_avoidance_controller_init()

    x = initial_x_mm
    y = initial_y_mm
    heading = math.radians(initial_heading_deg)
    velocity = 0.0
    steering = 0.0
    yaw_rate = 0.0
    prev_velocity = 0.0

    filtered_tof = None
    samples: list[RoverTrajectorySample] = []
    update_count = 0

    def ramp(actual: float, desired: float, limit: float) -> float:
        return actual + max(-limit, min(limit, desired - actual))

    sf_output = SoundFollowOutput()
    oa_output = ObstacleAvoidanceOutput()

    for now_ms in range(0, duration_ms, 50):
        # 1. Compute ToF distances
        raw_tof = []
        # Left (+94, +90, +side_angle), Center (0, 0, 0), Right (+94, -90, -side_angle)
        sensor_configs = (
            (94.0, 90.0, side_angle_deg),
            (0.0, 0.0, 0.0),
            (94.0, -90.0, -side_angle_deg),
        )
        for forward, lateral, angle in sensor_configs:
            origin_x = x + forward * math.cos(heading) - lateral * math.sin(heading)
            origin_y = y + forward * math.sin(heading) + lateral * math.cos(heading)
            ray_angle = heading + math.radians(angle)
            vx = math.cos(ray_angle)
            vy = math.sin(ray_angle)

            # Distance to obstacle
            d_obs = intersect_ray_circle(
                origin_x, origin_y, vx, vy, obstacle_x_mm, obstacle_y_mm, obstacle_radius_mm
            )
            # Distance to room walls
            d_wall = intersect_ray_room(origin_x, origin_y, vx, vy)
            d_min = min(d_obs, d_wall)
            raw_tof.append(max(1.0, min(4000.0, d_min)))

        filtered_tof = (
            raw_tof
            if filtered_tof is None
            else [(3.0 * old + new) / 4.0 for old, new in zip(filtered_tof, raw_tof)]
        )

        # 2. Sound Source DoA calculation
        dx_sound = sound_x_mm - x
        dy_sound = sound_y_mm - y
        dist_sound = math.hypot(dx_sound, dy_sound)
        world_sound_angle = math.atan2(dy_sound, dx_sound)
        # Relative bearing in ROS (CCW positive): angle = world - heading
        rel_sound_angle = world_sound_angle - heading
        while rel_sound_angle > math.pi:
            rel_sound_angle -= 2.0 * math.pi
        while rel_sound_angle < -math.pi:
            rel_sound_angle += 2.0 * math.pi
        # Rover coordinate: Right is positive (CW positive)
        rover_doa_deg = -math.degrees(rel_sound_angle)

        # XVF raw DoA: 0..359 (XVF clockwise positive == 0 in config, so raw = -rover_doa % 360)
        raw_xvf_doa = int(round(-rover_doa_deg)) % 360

        # Step controllers at 100ms
        if now_ms % 100 == 0:
            update_count += 1
            snapshot = sensor_snapshot(
                left_mm=round(filtered_tof[0]),
                center_mm=round(filtered_tof[1]),
                right_mm=round(filtered_tof[2]),
            )
            snapshot.gyro_dps_x10[2] = round(math.degrees(yaw_rate) * 10)
            snapshot.update_count = update_count
            # Forward acceleration from velocity change (1g = 9810 mm/s², dt=0.1s)
            accel_y_mg = int((velocity - prev_velocity) / 0.1 / 9.810)
            snapshot.accel_mg[1] = max(-700, min(700, accel_y_mg))
            prev_velocity = velocity

            obs = AcousticObservation(
                doa_deg=raw_xvf_doa,
                raw_doa_deg=raw_xvf_doa,
                level_dbfs_x100=-3000,
                peak_dbfs_x100=-2800,
                vad=1,
                doa_confidence=90,
                xvf_status=ACOUSTIC_XVF_STATUS_READY,
                audio_flags=0,
                xvf_raw_status=0,
                reserved=0,
                audio_frame_count=update_count,
                sample_sequence=update_count,
            )

            sf_input = SoundFollowInput(
                link_ready=1,
                new_observation=1,
                fault_active=0,
                motion_allowed=1,
                observation=obs,
                match_required=0,
                target_sound_matched=1,
                navigation_target_valid=0,
                navigation_bearing_deg=0,
                arrival_verify=0,
                arrived=0,
                avoidance_relisten=0,
                restart_request=0,
                imu_valid=1,
                gyro_z_dps_x10=snapshot.gyro_dps_x10[2],
                imu_update_count=update_count,
                pose_heading_valid=1,
                pose_heading_mrad=round(-heading * 1000.0),
            )

            handle.sound_follow_controller_step(ctypes.byref(sf_input), 100, ctypes.byref(sf_output))

            # Target steering from sound
            target_steer = sf_output.target_bearing_deg
            target_steer = max(-45, min(45, target_steer))

            handle.obstacle_avoidance_controller_step(
                ctypes.byref(snapshot),
                False,
                now_ms,
                ctypes.c_int16(target_steer),
                ctypes.c_int16(int(velocity)),
                ctypes.byref(oa_output),
            )

        # Merge outputs (same priority logic as task_think.c)
        is_spin_turn = False
        cmd_left_rpm = 0
        cmd_right_rpm = 0
        cmd_steer_deg = 0.0

        if oa_output.avoidance_in_progress and oa_output.is_spin_turn:
            # Escape pivot turn
            is_spin_turn = True
            cmd_left_rpm = oa_output.left_rpm
            cmd_right_rpm = oa_output.right_rpm
            cmd_steer_deg = 0.0
        elif oa_output.avoidance_in_progress and oa_output.actuator_enable:
            # Forward avoidance steering
            cmd_steer_deg = float(oa_output.steering_deg)
            cmd_left_rpm = oa_output.left_rpm
            cmd_right_rpm = oa_output.right_rpm
        elif sf_output.is_spin_turn and sf_output.actuator_enable:
            # Sound follow spin turn
            is_spin_turn = True
            cmd_left_rpm = sf_output.left_rpm
            cmd_right_rpm = sf_output.right_rpm
            cmd_steer_deg = 0.0
        elif sf_output.actuator_enable:
            # Normal sound follow
            cmd_steer_deg = float(sf_output.steering_deg)
            cmd_left_rpm = sf_output.left_rpm
            cmd_right_rpm = sf_output.right_rpm

        # Dynamics update
        target_velocity = (cmd_left_rpm + cmd_right_rpm) / 200.0 * speed_mm_s if not is_spin_turn else 0.0
        velocity = ramp(velocity, target_velocity, 40.0)
        steering = ramp(steering, cmd_steer_deg, 10.0)

        if is_spin_turn:
            # cmd_left_rpm > cmd_right_rpm is right turn (CW, negative yaw_rate)
            # cmd_left_rpm < cmd_right_rpm is left turn (CCW, positive yaw_rate)
            target_yaw_rate = -math.radians(turn_rate_dps) * (cmd_left_rpm - cmd_right_rpm) / 200.0
        else:
            # steering > 0 is right steer (CW, negative yaw_rate)
            # steering < 0 is left steer (CCW, positive yaw_rate)
            target_yaw_rate = (
                -math.radians(turn_rate_dps)
                * math.tan(math.radians(steering))
                * velocity
                / speed_mm_s
                if speed_mm_s > 0
                else 0.0
            )
        yaw_rate = ramp(yaw_rate, target_yaw_rate, math.radians(5.0))
        heading += yaw_rate * 0.05
        x += velocity * math.cos(heading) * 0.05
        y += velocity * math.sin(heading) * 0.05

        # Clearance calculation
        # Transform obstacle center to rover local frame
        dx_obs = obstacle_x_mm - x
        dy_obs = obstacle_y_mm - y
        local_obs_x = dx_obs * math.cos(heading) + dy_obs * math.sin(heading)
        local_obs_y = -dx_obs * math.sin(heading) + dy_obs * math.cos(heading)
        clamped_x = max(-300.0, min(150.0, local_obs_x))
        clamped_y = max(-130.0, min(130.0, local_obs_y))
        dist_to_obs_center = math.hypot(local_obs_x - clamped_x, local_obs_y - clamped_y)
        obs_clearance = dist_to_obs_center - obstacle_radius_mm

        # Wall clearance
        wall_clearance = min(
            x - (-450.0) - 300.0,
            2650.0 - x - 150.0,
            y - (-1550.0) - 130.0,
            1550.0 - y - 130.0,
        )

        samples.append(
            RoverTrajectorySample(
                time_ms=now_ms,
                x_mm=x,
                y_mm=y,
                heading_deg=math.degrees(heading),
                steering_deg=cmd_steer_deg,
                left_rpm=cmd_left_rpm,
                right_rpm=cmd_right_rpm,
                is_spin_turn=is_spin_turn,
                obstacle_clearance_mm=obs_clearance,
                wall_clearance_mm=wall_clearance,
                distance_to_sound_mm=dist_sound,
                doa_relative_deg=rover_doa_deg,
                state=sf_output.state,
                rule=oa_output.rule,
            )
        )

        # Stop if collision occurred
        if obs_clearance <= 0 or wall_clearance <= 0:
            break
        # Success if reached near sound source
        if dist_sound < 400.0:
            break

    return samples
