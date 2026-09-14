"""Run the actual CPU0 command task through an IPC outage and recovery."""
from pathlib import Path
import subprocess


def test_command_ipc_fault_recovers_only_after_safe_acknowledgements(tmp_path):
    sim = Path(__file__).resolve().parents[1]
    cpu0 = sim.parents[1] / "firmware/ra8p1/SoundExplorationRover_CPU0/src"
    executable = tmp_path / "command_recovery"
    subprocess.run([
        "clang", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O1", "-g",
        "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
        f"-I{sim / 'cpu0/shim'}", f"-I{cpu0}",
        str(cpu0 / "tasks/task_command.c"), str(sim / "cpu0/command_recovery_test.c"),
        "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True, timeout=10)
