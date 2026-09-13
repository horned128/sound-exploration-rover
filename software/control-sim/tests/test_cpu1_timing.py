"""Actual CPU1 time consumers; fake only the kernel, hardware and IPC boundaries."""
from pathlib import Path
import subprocess
import pytest


@pytest.mark.parametrize("feedback_enabled", [False, True])
def test_cpu1_elapsed_time_and_stop_paths(tmp_path, feedback_enabled):
    sim = Path(__file__).resolve().parents[1]
    cpu1 = sim.parents[1] / "firmware/ra8p1/SoundExplorationRover_CPU1/src"
    sources = [cpu1 / name for name in (
        "drivers/encoder.c", "services/drive_service.c",
        "services/actuator_service.c", "tasks/task_actuator.c")]
    config = tmp_path / "config"
    config.mkdir()
    text = (cpu1 / "config/drive_config.h").read_text()
    if feedback_enabled:
        text = text.replace("#define DRIVE_SPEED_FEEDBACK_ENABLE        (0U)",
                            "#define DRIVE_SPEED_FEEDBACK_ENABLE        (1U)")
    (config / "drive_config.h").write_text(text)
    executable = tmp_path / "cpu1_timing"
    subprocess.run([
        "clang", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O1", "-g",
        "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
        f"-I{tmp_path}", f"-I{sim / 'cpu1/shim'}", f"-I{cpu1}",
        *map(str, sources), str(sim / "cpu1/timing_test.c"), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)


def test_cpu1_status_schedule_and_packet_integrity(tmp_path):
    sim = Path(__file__).resolve().parents[1]
    cpu1 = sim.parents[1] / "firmware/ra8p1/SoundExplorationRover_CPU1/src"
    executable = tmp_path / "cpu1_status"
    subprocess.run([
        "clang", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O1", "-g",
        "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
        f"-I{sim / 'cpu1/shim'}", f"-I{cpu1}",
        str(cpu1 / "tasks/task_status.c"), str(sim / "cpu1/status_test.c"),
        "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
