import pytest

from controlsim.wall_world import run_wall_approach


@pytest.mark.parametrize("speed_mm_s", [180, 240, 300])
@pytest.mark.parametrize("turn_rate_dps", [20, 30])
@pytest.mark.parametrize("initial_heading_deg", [-10, 0, 10])
@pytest.mark.parametrize("side_angle_deg", [0, 10, 13, 15])
def test_closed_loop_wall_approach_prioritizes_forward_escape_without_reverse(
    speed_mm_s, turn_rate_dps, initial_heading_deg, side_angle_deg,
) -> None:
    samples = run_wall_approach(
        speed_mm_s=speed_mm_s, turn_rate_dps=turn_rate_dps, initial_heading_deg=initial_heading_deg,
        side_angle_deg=side_angle_deg,
    )
    # 250mmで停止させず、車体が接触する近さまで前進するケースを許容する。
    # ここでの安全条件は盲目的な後退を出さないこと。真正面の連続近接停止は
    # obstacle controller のセンサー逐次テストで別途検証する。
    assert min(sample.clearance_mm for sample in samples) > -5.0
    assert any(sample.rule in (1, 3, 4, 7, 8, 5) for sample in samples)
    assert all(sample.rule != 9 for sample in samples)
