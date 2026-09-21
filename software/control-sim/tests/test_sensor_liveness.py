"""Run the actual thinking task and controllers with frozen valid sensor data."""
from pathlib import Path
import subprocess
import pytest


@pytest.mark.parametrize("sensor_mode", [False, True])
def test_sensor_liveness_stop_and_recovery(tmp_path, sensor_mode):
    sim = Path(__file__).resolve().parents[1]
    cpu0 = sim.parents[1] / "firmware/ra8p1/SoundExplorationRover_CPU0/src"
    config = tmp_path / "config"
    config.mkdir()
    text = (cpu0 / "config/control_config.h").read_text()
    if sensor_mode:
        text = text.replace(
            "#define CPU0_AUTONOMY_MODE                 (CPU0_AUTONOMY_MODE_SOUND_FOLLOW)",
            "#define CPU0_AUTONOMY_MODE                 (CPU0_AUTONOMY_MODE_SENSOR_RULE)")
    (config / "control_config.h").write_text(text)
    executable = tmp_path / "liveness"
    subprocess.run([
        "clang", "-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-O1", "-g",
        "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
        f"-I{tmp_path}", f"-I{sim / 'cpu0/shim'}", f"-I{cpu0}",
        str(cpu0 / "tasks/task_think.c"), str(cpu0 / "control/sound_follow_controller.c"),
        str(cpu0 / "control/obstacle_avoidance_controller.c"),
        str(cpu0 / "control/control_mlp_planner.c"),
        str(cpu0 / "control/safety_arbiter.c"),
        str(sim / "shim/tflm_runtime_shim.c"),
        str(sim / "cpu0/liveness_test.c"), "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True, timeout=10)
