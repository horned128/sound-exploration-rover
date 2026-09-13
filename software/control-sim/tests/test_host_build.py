import subprocess
import sys
from pathlib import Path


def test_portable_control_code_runs_under_asan_and_ubsan() -> None:
    control_sim_root = Path(__file__).resolve().parents[1]
    subprocess.run([sys.executable, str(control_sim_root / "build.py"), "--sanitized-smoke"], check=True)
