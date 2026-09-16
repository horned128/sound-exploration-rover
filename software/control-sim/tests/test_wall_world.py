import math

import pytest

from controlsim.wall_world import run_wall_approach


@pytest.mark.parametrize("speed_mm_s", [180, 240, 300])
@pytest.mark.parametrize("turn_rate_dps", [20, 30])
@pytest.mark.parametrize("initial_heading_deg", [-10, 0, 10])
@pytest.mark.parametrize("side_angle_deg", [-5, 0, 5])
def test_closed_loop_wall_approach_clears_the_wall_without_repeating_backup(
    speed_mm_s, turn_rate_dps, initial_heading_deg, side_angle_deg,
) -> None:
    samples = run_wall_approach(
        speed_mm_s=speed_mm_s, turn_rate_dps=turn_rate_dps, initial_heading_deg=initial_heading_deg,
        side_angle_deg=side_angle_deg,
    )
    assert min(sample.clearance_mm for sample in samples) > 0
    assert math.hypot(samples[-1].x_mm + 900, samples[-1].y_mm) > 3000
    assert all(sample.rule == 1 for sample in samples[-100:])
    backup_entries = sum(
        sample.rule == 9 and (i == 0 or samples[i - 1].rule != 9)
        for i, sample in enumerate(samples)
    )
    assert backup_entries <= 3
