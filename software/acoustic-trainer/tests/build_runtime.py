"""Build the actual CPU0 C wrapper and unchanged reference TFLM on the host."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[3]
VENDOR = ROOT / "firmware/ra8p1/common/tflm"
CPU0 = ROOT / "firmware/ra8p1/SoundExplorationRover_CPU0/src"
BUILD = Path(__file__).resolve().parents[1] / "build"


def build():
    BUILD.mkdir(exist_ok=True)
    includes = [VENDOR, VENDOR / "third_party/flatbuffers/include",
                VENDOR / "third_party/gemmlowp", VENDOR / "third_party/ruy",
                VENDOR / "third_party/kissfft", CPU0,
                ROOT / "software/control-sim/shim"]
    common = ["-O2", "-g", "-fPIC", "-DTF_LITE_STATIC_MEMORY", "-DTF_LITE_DISABLE_X86_NEON",
              "-DTF_LITE_STRIP_ERROR_STRINGS", "-DNDEBUG",
              "-fno-unwind-tables", "-fno-asynchronous-unwind-tables"]
    common += [f"-I{p}" for p in includes]
    sources = sorted(VENDOR.rglob("*.cc")) + sorted(VENDOR.rglob("*.c"))
    sources += [CPU0 / "ai/tflm_runtime.cc"]

    def compile_one(source):
        obj = BUILD / "objects" / source.relative_to(ROOT).with_suffix(".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        cpp = source.suffix == ".cc"
        command = (["c++", "-std=c++17", "-fno-exceptions", "-fno-rtti", "-fno-threadsafe-statics"]
                   if cpp else ["cc", "-std=c99"])
        result = subprocess.run(command + common + ["-c", str(source), "-o", str(obj)],
                                capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(f"{source}\n{result.stderr}")
        return obj

    with ThreadPoolExecutor(max_workers=8) as pool:
        objects = list(pool.map(compile_one, sources))
    output = BUILD / ("libtflm_runtime.dylib" if platform.system() == "Darwin" else "libtflm_runtime.so")
    subprocess.run(["c++", "-shared", *map(str, objects), "-o", str(output)], check=True)
    print(f"Built {len(sources)} sources: {output}")
    return output


if __name__ == "__main__":
    build()
