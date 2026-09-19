"""Small closed-loop regression world, not a calibrated vehicle simulator.

The actual firmware controller drives a planar 4WS arc model toward x=0.
Assumptions: 150 mm front overhang, 260 mm width, side ToFs at (+94, +/-90)
mm and all three looking forward, 50 ms sensing with the firmware's 3:1 filter. Motor
and servo slew limits model response lag. Missing ray intersections return
4000 mm as a valid clear reading; real ToF invalid-range handling is separate.
Use this to detect loops and contact regressions, not to certify clearance.
"""

from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass
from typing import Callable

from .bindings import ObstacleAvoidanceOutput, SensorSnapshot, library
from .scenarios import sensor_snapshot


@dataclass(frozen=True)
class WallSample:
    time_ms: int
    x_mm: float
    y_mm: float
    heading_deg: float
    rule: int
    clearance_mm: float


ControllerStep = Callable[[SensorSnapshot, int], ObstacleAvoidanceOutput]


def run_wall_approach(
    *, speed_mm_s: float = 240, turn_rate_dps: float = 20,
    initial_heading_deg: float = 0, duration_ms: int = 30000,
    side_angle_deg: float = 0,
    controller_step: ControllerStep | None = None,
) -> list[WallSample]:
    """Feed simulated ToF and IMU observations back into the real C controller.

    speed_mm_s and turn_rate_dps are the response at 100 commanded RPM and
    45 degrees steering, deliberately varied independently to stress slip.
    An optional controller_step permits comparison against a saved baseline.
    """
    if controller_step is None:
        handle = library()
        handle.obstacle_avoidance_controller_init()

        def controller_step(snapshot: SensorSnapshot, now_ms: int) -> ObstacleAvoidanceOutput:
            output = ObstacleAvoidanceOutput()
            handle.obstacle_avoidance_controller_step(
                ctypes.byref(snapshot), False, now_ms, ctypes.c_int16(0), ctypes.byref(output)
            )
            return output

    x, y, heading = -900.0, 0.0, math.radians(initial_heading_deg)
    velocity = steering = yaw_rate = 0.0
    filtered = None
    output = ObstacleAvoidanceOutput()
    samples = []

    def ramp(actual: float, desired: float, limit: float) -> float:
        return actual + max(-limit, min(limit, desired - actual))

    for now_ms in range(0, duration_ms, 50):
        raw = []
        for forward, lateral, angle in ((94, -90, -side_angle_deg), (0, 0, 0), (94, 90, side_angle_deg)):
            origin_x = x + forward * math.cos(heading) - lateral * math.sin(heading)
            ray_x = math.cos(heading + math.radians(angle))
            raw.append(max(1, min(4000, -origin_x / ray_x)) if ray_x > 0 else 4000)
        filtered = raw if filtered is None else [(3 * old + new) / 4 for old, new in zip(filtered, raw)]
        if now_ms % 100 == 0:
            snapshot = sensor_snapshot(
                left_mm=round(filtered[0]), center_mm=round(filtered[1]), right_mm=round(filtered[2])
            )
            snapshot.gyro_dps_x10[2] = round(math.degrees(yaw_rate) * 10)
            output = controller_step(snapshot, now_ms)

        target_velocity = (output.left_rpm + output.right_rpm) / 200 * speed_mm_s if output.actuator_enable else 0
        velocity = ramp(velocity, target_velocity, 40)
        steering = ramp(steering, output.steering_deg, 10)
        yaw_rate = math.radians(turn_rate_dps) * math.tan(math.radians(steering)) * velocity / speed_mm_s
        heading += yaw_rate * 0.05
        x += velocity * math.cos(heading) * 0.05
        y += velocity * math.sin(heading) * 0.05
        clearance = min(
            -x - forward * math.cos(heading) + lateral * math.sin(heading)
            for forward in (-300, 150) for lateral in (-130, 130)
        )
        samples.append(WallSample(now_ms, x, y, math.degrees(heading), output.rule, clearance))
        if clearance <= 0:
            break
    return samples
