from controlsim.bindings import obstacle_avoidance_step
from controlsim.scenarios import frontal_wall_approach


def test_front_wall_stops_at_existing_hard_stop_distance() -> None:
    outputs = [obstacle_avoidance_step(snapshot) for snapshot in frontal_wall_approach()]

    assert all(output.actuator_enable for output in outputs[:-1])
    assert not outputs[-1].actuator_enable
    assert outputs[-1].left_rpm == 0
    assert outputs[-1].right_rpm == 0
    assert not outputs[-1].emergency_stop

