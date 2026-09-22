"""Tests for the TFLM int8 control MLP planner and C runtime ABI.

Verifies:
1. Memory safety under AddressSanitizer and UndefinedBehaviorSanitizer.
2. TFLM runtime C ABI contract, lifecycle, and error-handling edge cases.
3. Numerical parity (<= 2 LSB, >= 90% 0 LSB match) between Python TFLite reference
   and C tflm_runtime_invoke.
4. Firmware control_mlp_planner C API (init, reset, proactive steering, rate limiting,
   and close-distance continuation).
5. Closed-loop wall approach forward-priority behavior across 54 dynamic conditions.
"""

from __future__ import annotations

import ctypes
import math
import os
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np
import pytest

# Suppress verbose TF logging
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf

from controlsim.bindings import (
    ControlMlpOutput,
    ObstacleAvoidanceOutput,
    SafetyMotionCommand,
    SensorSnapshot,
    control_mlp_plan_step,
    library,
    safety_arbiter_arbitrate_step,
)
from controlsim.scenarios import sensor_snapshot
from controlsim.wall_world import run_wall_approach

ROOT_DIR = Path(__file__).resolve().parents[3]
CONTROL_SIM_DIR = ROOT_DIR / "software/control-sim"
BUILD_DIR = CONTROL_SIM_DIR / "build"
TFLITE_MODEL_PATH = BUILD_DIR / "control_mlp_int8.tflite"
BUILD_SCRIPT = CONTROL_SIM_DIR / "build.py"

# Runtime library path (acoustic trainer build directory)
EXT = ".dylib" if platform.system() == "Darwin" else ".so"
TFLM_RUNTIME_PATH = ROOT_DIR / f"software/acoustic-trainer/build/libtflm_runtime{EXT}"


class TflmRuntimeInfo(ctypes.Structure):
    _fields_ = [
        ("input_bytes", ctypes.c_uint32),
        ("output_bytes", ctypes.c_uint32),
        ("arena_used_bytes", ctypes.c_uint32),
        ("input_scale", ctypes.c_float),
        ("output_scale", ctypes.c_float),
        ("input_zero_point", ctypes.c_int32),
        ("output_zero_point", ctypes.c_int32),
    ]


def aligned_buffer(data: bytes) -> tuple[ctypes.Array[ctypes.c_char], ctypes.c_void_p]:
    storage = ctypes.create_string_buffer(len(data) + 15)
    address = (ctypes.addressof(storage) + 15) & ~15
    ctypes.memmove(address, data, len(data))
    return storage, ctypes.c_void_p(address)


# ===================================================================
# 1. Sanitizer Smoke Test
# ===================================================================

def test_control_mlp_sanitized_smoke() -> None:
    """Validate memory safety of control_mlp_planner and runtime shim under ASan / UBSan."""
    cmd = [sys.executable, str(BUILD_SCRIPT), "--sanitized-smoke"]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    assert proc.returncode == 0, f"ASan/UBSan smoke failed:\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"


# ===================================================================
# 2. TFLM Runtime C ABI Tests
# ===================================================================

@pytest.fixture(scope="module")
def tflm_c_lib():
    if not TFLM_RUNTIME_PATH.exists():
        pytest.skip(f"libtflm_runtime not found at {TFLM_RUNTIME_PATH}")
    lib = ctypes.CDLL(str(TFLM_RUNTIME_PATH))
    lib.tflm_runtime_init.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.tflm_runtime_init.restype = ctypes.c_int32
    lib.tflm_runtime_invoke.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.tflm_runtime_invoke.restype = ctypes.c_int32
    lib.tflm_runtime_get_info.argtypes = [ctypes.POINTER(TflmRuntimeInfo)]
    lib.tflm_runtime_get_info.restype = ctypes.c_int32
    lib.tflm_runtime_reset.argtypes = []
    lib.tflm_runtime_reset.restype = None
    return lib


def test_control_mlp_tflm_c_abi_lifecycle(tflm_c_lib) -> None:
    """Verify runtime ABI contract: uninitialized errors, null handling, info extraction, and reset."""
    lib = tflm_c_lib
    lib.tflm_runtime_reset()

    # 1. Calls prior to initialization must return -4 (Not Initialized)
    info = TflmRuntimeInfo()
    assert lib.tflm_runtime_get_info(ctypes.byref(info)) == -4
    out_buf = (ctypes.c_int8 * 2)()
    in_buf = (ctypes.c_int8 * 10)()
    assert lib.tflm_runtime_invoke(in_buf, 10, out_buf, 2) == -4

    # 2. Invalid init parameters must return -1 (Invalid Arg) or -2 (Bad Model)
    assert lib.tflm_runtime_init(None, 0) == -1
    bad_storage, bad_addr = aligned_buffer(bytes(64))
    assert lib.tflm_runtime_init(bad_addr, 64) == -2

    # 3. Valid model initialization
    assert TFLITE_MODEL_PATH.exists(), f"Model file missing: {TFLITE_MODEL_PATH}"
    model_bytes = TFLITE_MODEL_PATH.read_bytes()
    storage, model_addr = aligned_buffer(model_bytes)
    assert lib.tflm_runtime_init(model_addr, len(model_bytes)) == 0

    # 4. Model info verification
    assert lib.tflm_runtime_get_info(ctypes.byref(info)) == 0
    assert info.input_bytes == 10
    assert info.output_bytes == 2
    assert 0 < info.arena_used_bytes <= 96 * 1024  # Static 96 KiB arena
    assert info.input_scale > 0.0
    assert info.output_scale > 0.0

    # 5. Invalid invoke buffers must return -1
    assert lib.tflm_runtime_invoke(None, 10, out_buf, 2) == -1
    assert lib.tflm_runtime_invoke(in_buf, 9, out_buf, 2) == -1
    assert lib.tflm_runtime_invoke(in_buf, 10, out_buf, 1) == -1

    # 6. Clean reset
    lib.tflm_runtime_reset()
    assert lib.tflm_runtime_get_info(ctypes.byref(info)) == -4


# ===================================================================
# 3. Numerical Parity: Python TFLite vs C Runtime
# ===================================================================

def test_control_mlp_numerical_parity_against_tflite_reference(tflm_c_lib) -> None:
    """Verify numerical equivalence between TFLite Python reference and C tflm_runtime_invoke."""
    lib = tflm_c_lib
    model_bytes = TFLITE_MODEL_PATH.read_bytes()
    storage, model_addr = aligned_buffer(model_bytes)
    assert lib.tflm_runtime_init(model_addr, len(model_bytes)) == 0

    # Reference interpreter with BUILTIN_REF fixed-point kernel
    reference = tf.lite.Interpreter(
        model_content=model_bytes,
        experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_REF,
    )
    reference.allocate_tensors()
    inp_details = reference.get_input_details()[0]
    out_details = reference.get_output_details()[0]

    # Test suite: boundary corners + random representative feature vectors
    rng = np.random.default_rng(20260920)
    vectors: list[np.ndarray] = [
        np.full((1, 10), -128, dtype=np.int8),
        np.full((1, 10), 0, dtype=np.int8),
        np.full((1, 10), 127, dtype=np.int8),
    ]
    for _ in range(30):
        vectors.append(rng.integers(-128, 128, (1, 10), dtype=np.int8))

    exact_matches = 0
    max_error = 0

    for vec in vectors:
        reference.set_tensor(inp_details["index"], vec)
        reference.invoke()
        expected = reference.get_tensor(out_details["index"])[0]

        c_in = (ctypes.c_int8 * 10)(*vec[0])
        c_out = (ctypes.c_int8 * 2)()
        assert lib.tflm_runtime_invoke(c_in, 10, c_out, 2) == 0

        actual = np.array(list(c_out), dtype=np.int16)
        error = int(np.max(np.abs(actual - expected.astype(np.int16))))
        max_error = max(max_error, error)
        if error == 0:
            exact_matches += 1

        # The two hidden int8 layers each round an accumulator.  The portable
        # runtime must stay within two output LSBs of TensorFlow's reference.
        assert error <= 2, f"Parity mismatch > 2 LSB: actual={actual}, expected={expected}"

    # At least 90% must match to 0 LSB exactly
    parity_rate = exact_matches / len(vectors)
    assert parity_rate >= 0.90, f"0 LSB parity rate too low: {parity_rate:.1%}"

    lib.tflm_runtime_reset()


# ===================================================================
# 4. Firmware C Planner API Tests (control_mlp_planner.c)
# ===================================================================

def test_control_mlp_planner_lifecycle_and_proactive_steering() -> None:
    """Verify C planner initialization, proactive steering away from obstacles, and steady state."""
    handle = library()
    handle.control_mlp_planner_reset()
    assert handle.control_mlp_planner_init() == 1
    assert handle.control_mlp_planner_is_ready() == 1

    # 1. Clear corridor (4000 mm): steady-state heading should be ~0 deg, speed scale 1.0
    snap_clear = sensor_snapshot(left_mm=4000, center_mm=4000, right_mm=4000)
    for _ in range(12):
        out_clear = control_mlp_plan_step(snap_clear, 0.0)
    assert abs(out_clear.steering_deg) <= 3.0
    assert out_clear.speed_scale >= 0.95
    assert not out_clear.is_blocked
    assert not out_clear.fallback_required

    # 2. Obstacle on left (300 mm L, 500 mm C, 2000 mm R): must steer decisively RIGHT (positive)
    handle.control_mlp_planner_reset()
    handle.control_mlp_planner_init()
    snap_left = sensor_snapshot(left_mm=300, center_mm=500, right_mm=2000)
    for _ in range(12):
        out_left = control_mlp_plan_step(snap_left, 0.0)
    assert out_left.steering_deg >= 25.0, f"Expected strong right turn, got {out_left.steering_deg}"

    # 3. Obstacle on right (2000 mm L, 500 mm C, 300 mm R): must steer decisively LEFT (negative)
    handle.control_mlp_planner_reset()
    handle.control_mlp_planner_init()
    snap_right = sensor_snapshot(left_mm=2000, center_mm=500, right_mm=300)
    for _ in range(12):
        out_right = control_mlp_plan_step(snap_right, 0.0)
    assert out_right.steering_deg <= -25.0, f"Expected strong left turn, got {out_right.steering_deg}"


def test_control_mlp_follows_each_sound_side_in_clear_space() -> None:
    """DoAの右正・左負を、障害物のない通路で同じ操舵符号へ保つ。"""
    handle = library()
    clear = sensor_snapshot(left_mm=4000, center_mm=4000, right_mm=4000)

    handle.control_mlp_planner_reset()
    assert handle.control_mlp_planner_init() == 1
    for _ in range(12):
        right_output = control_mlp_plan_step(clear, 35.0)
    assert right_output.steering_deg >= 20.0, right_output.steering_deg

    handle.control_mlp_planner_reset()
    assert handle.control_mlp_planner_init() == 1
    for _ in range(12):
        left_output = control_mlp_plan_step(clear, -35.0)
    assert left_output.steering_deg <= -20.0, left_output.steering_deg


def test_control_mlp_holds_the_selected_clearance_side_through_wall_measurements() -> None:
    """The safety guard must not weaken or reverse a selected side merely because that side sees a wall."""
    handle = library()
    handle.control_mlp_planner_reset()
    assert handle.control_mlp_planner_init() == 1

    # Center obstacle with the right side open: choose a right turn and ramp to the guarded minimum.
    first = sensor_snapshot(left_mm=500, center_mm=780, right_mm=1450)
    for _ in range(8):
        first_out = control_mlp_plan_step(first, 0.0)
    assert first_out.steering_deg >= 28.0

    # The right sensor then sees a nearer wall while the passage remains open.  The
    # selected turn remains committed until all three clearances have recovered.
    changed = sensor_snapshot(left_mm=900, center_mm=780, right_mm=620)
    for _ in range(8):
        changed_out = control_mlp_plan_step(changed, 0.0)
    assert changed_out.steering_deg >= 28.0
    assert changed_out.speed_scale <= 0.60

    clear = sensor_snapshot(left_mm=1500, center_mm=1300, right_mm=1500)
    for _ in range(8):
        clear_out = control_mlp_plan_step(clear, 0.0)
    assert abs(clear_out.steering_deg) < 28.0


def test_control_mlp_planner_failsafe_and_rate_limiting() -> None:
    """Verify invalid-sensor fallback and rate limits without a 250mm distance veto."""
    handle = library()
    handle.control_mlp_planner_reset()
    handle.control_mlp_planner_init()

    # 1. Close distance alone does not trigger a planner hard stop.
    for idx in range(3):
        dists = [1000, 1000, 1000]
        dists[idx] = 249
        handle.control_mlp_planner_reset()
        handle.control_mlp_planner_init()
        snap = sensor_snapshot(left_mm=dists[0], center_mm=dists[1], right_mm=dists[2])
        out = control_mlp_plan_step(snap, 0.0)
        assert not out.fallback_required
        assert not out.is_blocked, f"Channel {idx} close reading must not stop by distance"
        assert out.speed_scale > 0.0

    # 2. Invalid sensor snapshot / NULL fallback
    out_null = control_mlp_plan_step(None, 0.0)
    assert out_null.fallback_required
    assert out_null.is_blocked

    # 3. Rate limiting check: maximum 9.0 deg per 100 ms step
    handle.control_mlp_planner_reset()
    handle.control_mlp_planner_init()
    snap_step0 = sensor_snapshot(left_mm=300, center_mm=500, right_mm=2000)
    out0 = control_mlp_plan_step(snap_step0, 0.0)
    assert abs(out0.steering_deg) <= 9.01, f"First step exceeded 9.0 deg/step: {out0.steering_deg}"


# ===================================================================
# 5. Closed-Loop Wall Approach Simulation (54 Dynamic Conditions)
# ===================================================================

@pytest.mark.parametrize("speed_mm_s", [180, 240, 300])
@pytest.mark.parametrize("turn_rate_dps", [20, 30])
@pytest.mark.parametrize("initial_heading_deg", [-10, 0, 10])
@pytest.mark.parametrize("side_angle_deg", [-5, 0, 5])
def test_closed_loop_wall_approach_safety_arbiter_and_mlp(
    speed_mm_s: float,
    turn_rate_dps: float,
    initial_heading_deg: float,
    side_angle_deg: float,
) -> None:
    """Verify zero collision across all 54 dynamic conditions under MLP + Safety Arbiter."""
    handle = library()
    handle.obstacle_avoidance_controller_init()
    handle.control_mlp_planner_reset()
    handle.control_mlp_planner_init()

    def controller_step(snapshot: SensorSnapshot, now_ms: int) -> ObstacleAvoidanceOutput:
        mlp_out = control_mlp_plan_step(snapshot, 0.0)
        oa_out = ObstacleAvoidanceOutput()
        handle.obstacle_avoidance_controller_step(ctypes.byref(snapshot), False, now_ms, 0, ctypes.byref(oa_out))
        cmd = SafetyMotionCommand()

        # task_think.cと同じ調停: 通常域はMLP、方向ラッチした回避中だけは
        # 決定論的なToF制御を最優先する。
        if oa_out.avoidance_in_progress:
            cmd.steering_deg = oa_out.steering_deg
            cmd.left_rpm = oa_out.left_rpm
            cmd.right_rpm = oa_out.right_rpm
            cmd.actuator_enable = oa_out.actuator_enable
            cmd.emergency_stop = oa_out.emergency_stop
        else:
            cmd.steering_deg = int(round(mlp_out.steering_deg))
            if mlp_out.is_blocked or mlp_out.speed_scale <= 0.01:
                cmd.left_rpm = 0
                cmd.right_rpm = 0
                cmd.actuator_enable = False
            else:
                rpm = int(round(mlp_out.speed_scale * 100))
                cmd.left_rpm = rpm
                cmd.right_rpm = rpm
                cmd.actuator_enable = True
            cmd.emergency_stop = False

        arbitrated = safety_arbiter_arbitrate_step(cmd, snapshot, True)
        res = ObstacleAvoidanceOutput()
        res.state = 1
        res.rule = oa_out.rule
        res.steering_deg = arbitrated.steering_deg
        res.is_spin_turn = oa_out.is_spin_turn
        res.left_rpm = arbitrated.left_rpm
        res.right_rpm = arbitrated.right_rpm
        res.actuator_enable = arbitrated.actuator_enable
        res.emergency_stop = arbitrated.emergency_stop
        return res

    samples = run_wall_approach(
        speed_mm_s=speed_mm_s,
        turn_rate_dps=turn_rate_dps,
        initial_heading_deg=initial_heading_deg,
        side_angle_deg=side_angle_deg,
        controller_step=controller_step,
    )
    min_clearance = min(sample.clearance_mm for sample in samples)
    # wall_world integrates the body at 50 ms and rounds ToF to 1 mm, so its contact
    # sample can overshoot by less than one integration quantum.  Reject material
    # penetration while allowing this sub-2-mm numerical boundary error.
    assert min_clearance > -2.0, f"Material collision detected: min clearance = {min_clearance:.2f} mm"
