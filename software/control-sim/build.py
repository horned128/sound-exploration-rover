#!/usr/bin/env python3
"""Build the rover's portable control code for off-target tests."""

from __future__ import annotations

import argparse
import platform
import subprocess
from pathlib import Path


CONTROL_SIM_ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = CONTROL_SIM_ROOT.parents[1]
CPU0_SOURCE_ROOT = REPOSITORY_ROOT / "firmware/ra8p1/SoundExplorationRover_CPU0/src"
SHIM_ROOT = CONTROL_SIM_ROOT / "shim"
BUILD_ROOT = CONTROL_SIM_ROOT / "build"

CONTROLLER_SOURCES = (
    CPU0_SOURCE_ROOT / "control/sound_follow_controller.c",
    CPU0_SOURCE_ROOT / "control/obstacle_avoidance_controller.c",
    CPU0_SOURCE_ROOT / "control/smooth_avoidance_planner.c",
    CPU0_SOURCE_ROOT / "control/control_mlp_planner.c",
    CPU0_SOURCE_ROOT / "control/safety_arbiter.c",
    CPU0_SOURCE_ROOT / "services/acoustic_identifier.c",
    CPU0_SOURCE_ROOT / "services/background_model.c",
    CPU0_SOURCE_ROOT / "services/odometry.c",
    SHIM_ROOT / "tflm_runtime_shim.c",
)

FEATURE_PROTOCOL_SOURCES = (
    REPOSITORY_ROOT / "firmware/common/acoustic_protocol.c",
    CPU0_SOURCE_ROOT / "services/acoustic_feature_assembler.c",
)


def library_path() -> Path:
    extension = ".dylib" if platform.system() == "Darwin" else ".so"
    return BUILD_ROOT / f"libcontrolsim{extension}"


def compile_command(*, shared: bool, sanitized: bool, output: Path, sources: tuple[Path, ...]) -> list[str]:
    command = [
        "clang",
        "-std=c99",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-O0",
        "-g",
        f"-I{SHIM_ROOT}",
        f"-I{CPU0_SOURCE_ROOT}",
    ]
    if shared:
        command.extend(["-fPIC", "-shared"])
    if sanitized:
        command.append("-fsanitize=address,undefined")
    command.extend(str(source) for source in sources)
    command.extend(["-lm", "-o", str(output)])
    return command


def run(command: list[str]) -> None:
    subprocess.run(command, check=True, cwd=REPOSITORY_ROOT)


def build_library() -> Path:
    """Build the ctypes library.

    macOS will not load a sanitizer-instrumented dylib into the framework Python
    process. Sanitizers are therefore exercised by ``run_sanitized_smoke``;
    the shared library stays uninstrumented solely for the Python FFI boundary.
    """

    BUILD_ROOT.mkdir(exist_ok=True)
    output = library_path()
    run(compile_command(shared=True, sanitized=False, output=output, sources=CONTROLLER_SOURCES))
    return output


def run_layout_probe() -> str:
    BUILD_ROOT.mkdir(exist_ok=True)
    executable = BUILD_ROOT / "layout_probe"
    source = CONTROL_SIM_ROOT / "controlsim/layout_probe.c"
    run(compile_command(shared=False, sanitized=True, output=executable, sources=(source,)))
    return subprocess.run(
        [str(executable)], check=True, cwd=REPOSITORY_ROOT, capture_output=True, text=True
    ).stdout


def run_sanitized_smoke() -> None:
    BUILD_ROOT.mkdir(exist_ok=True)
    executable = BUILD_ROOT / "sanitized_smoke"
    source = CONTROL_SIM_ROOT / "controlsim/sanitized_smoke.c"
    run(
        compile_command(
            shared=False,
            sanitized=True,
            output=executable,
            sources=(*CONTROLLER_SOURCES, source),
        )
    )
    run([str(executable)])


def run_feature_protocol_test() -> None:
    BUILD_ROOT.mkdir(exist_ok=True)
    executable = BUILD_ROOT / "feature_protocol_test"
    source = CONTROL_SIM_ROOT / "controlsim/feature_protocol_test.c"
    run(
        compile_command(
            shared=False,
            sanitized=True,
            output=executable,
            sources=(*FEATURE_PROTOCOL_SOURCES, source),
        )
    )
    run([str(executable)])


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--layout", action="store_true", help="build and run the C layout probe")
    mode.add_argument("--sanitized-smoke", action="store_true", help="run portable code under ASan and UBSan")
    mode.add_argument("--feature-protocol", action="store_true", help="test feature protocol and reassembly")
    arguments = parser.parse_args()

    if arguments.layout:
        print(run_layout_probe(), end="")
    elif arguments.sanitized_smoke:
        run_sanitized_smoke()
    elif arguments.feature_protocol:
        run_feature_protocol_test()
    else:
        print(build_library())


if __name__ == "__main__":
    main()
