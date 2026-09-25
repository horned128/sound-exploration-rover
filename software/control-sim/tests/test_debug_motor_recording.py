"""Exercise the production header's switch/timer policy with sanitizers."""
from build import run_debug_motor_recording_test


def test_debug_motor_short_long_press_timeout_and_veto() -> None:
    run_debug_motor_recording_test()
