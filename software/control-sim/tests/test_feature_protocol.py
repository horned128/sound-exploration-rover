import subprocess
import sys
from pathlib import Path


def test_feature_protocol_and_reassembly_under_sanitizers() -> None:
    control_sim_root = Path(__file__).resolve().parents[1]
    subprocess.run([sys.executable, str(control_sim_root / "build.py"), "--feature-protocol"], check=True)
