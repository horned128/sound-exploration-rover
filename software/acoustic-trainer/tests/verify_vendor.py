"""Verify the checksummed, unmodified TFLM export and its license files."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[3] / "firmware/ra8p1/common/tflm"
manifest = json.loads((root / "SHA256SUMS.json").read_text())
for name, expected in manifest.items():
    actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit(f"Vendor checksum mismatch: {name}")
actual_files = {str(path.relative_to(root)) for path in root.rglob("*") if path.is_file()}
assert actual_files == set(manifest) | {"README.md", "SHA256SUMS.json"}
print(f"Verified {len(manifest)} vendor files and licenses")
