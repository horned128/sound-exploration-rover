"""Train int8 obstacle avoidance MLP via policy distillation and export C header.

Collects rollouts from the closed-loop wall_world simulator, blends real telemetry
from fixtures/wall_loop_20260915.jsonl, trains a 10->16->2 MLP, quantizes to full int8,
and generates control_mlp_model.h for CPU0 firmware.
"""

from __future__ import annotations

import ctypes
import json
import math
from pathlib import Path

import numpy as np

# Suppress verbose TF logging before importing TensorFlow
import os
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf

import sys
CONTROL_SIM_DIR = Path(__file__).resolve().parents[1]
if str(CONTROL_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROL_SIM_DIR))

try:
    from controlsim.bindings import (
        library, SensorSnapshot, ObstacleAvoidanceOutput,
        SmoothAvoidanceInput, smooth_avoidance_plan_step,
    )
    from controlsim.scenarios import sensor_snapshot
    from controlsim.wall_world import run_wall_approach
except ImportError:
    from .bindings import (
        library, SensorSnapshot, ObstacleAvoidanceOutput,
        SmoothAvoidanceInput, smooth_avoidance_plan_step,
    )
    from .scenarios import sensor_snapshot
    from .wall_world import run_wall_approach

ROOT_DIR = Path(__file__).resolve().parents[3]
CPU0_SRC_DIR = ROOT_DIR / "firmware/ra8p1/SoundExplorationRover_CPU0/src"
HEADER_OUTPUT_PATH = CPU0_SRC_DIR / "control/control_mlp_model.h"
BUILD_DIR = CONTROL_SIM_DIR / "build"
TFLITE_OUTPUT_PATH = BUILD_DIR / "control_mlp_int8.tflite"
FIXTURE_PATH = CONTROL_SIM_DIR / "tests/fixtures/wall_loop_20260915.jsonl"


def normalize_input(
    tof_mm: list[float] | tuple[float, ...],
    tof_valid: list[bool] | tuple[bool, ...],
    target_heading_deg: float,
    current_speed_scale: float,
    yaw_rate_dps: float,
) -> np.ndarray:
    """Construct 10-dimensional normalized input vector:

    [0..2]: ToF L, C, R distances normalized (0..4000 mm -> 0.0..1.0, invalid = 1.0)
    [3..5]: ToF L, C, R validity flags (0.0 or 1.0)
    [6..7]: Target heading sin(theta), cos(theta)
    [8]:    Current speed scale (0.0..1.0)
    [9]:    Yaw rate normalized (yaw_dps / 50.0, clamped to [-1.0, 1.0])
    """
    x = np.zeros(10, dtype=np.float32)
    for i in range(3):
        if tof_valid[i]:
            x[i] = np.clip(tof_mm[i] / 4000.0, 0.0, 1.0)
            x[3 + i] = 1.0
        else:
            x[i] = 1.0
            x[3 + i] = 0.0

    rad = math.radians(target_heading_deg)
    x[6] = math.sin(rad)
    x[7] = math.cos(rad)
    x[8] = np.clip(current_speed_scale, 0.0, 1.0)
    x[9] = np.clip(yaw_rate_dps / 50.0, -1.0, 1.0)
    return x


def collect_simulator_rollouts(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Simulate approaches toward wall using expert controller and record (input, target) pairs."""
    inputs: list[np.ndarray] = []
    targets: list[np.ndarray] = []

    speeds = [180, 240, 300]
    turn_rates = [20, 25, 30]
    initial_headings = [-15, -10, -5, 0, 5, 10, 15]
    side_angles = [-5, 0, 5]

    for speed in speeds:
        for turn_rate in turn_rates:
            for init_head in initial_headings:
                for side_ang in side_angles:
                    handle = library()
                    handle.obstacle_avoidance_controller_init()

                    current_speed = 0.0

                    def step_wrapper(snapshot: SensorSnapshot, now_ms: int) -> ObstacleAvoidanceOutput:
                        nonlocal current_speed
                        out = ObstacleAvoidanceOutput()
                        handle.obstacle_avoidance_controller_step(
                            ctypes.byref(snapshot), False, now_ms, ctypes.c_int16(0), ctypes.byref(out)
                        )
                        # We distill forward/turning proactive actions (rules: 1, 2, 3, 4)
                        if out.rule in (1, 2, 3, 4):
                            steer_norm = float(out.steering_deg) / 45.0
                            c_dist = float(snapshot.tof_distance_mm[1])
                            l_dist = float(snapshot.tof_distance_mm[0])
                            r_dist = float(snapshot.tof_distance_mm[2])
                            yaw_dps = float(snapshot.gyro_dps_x10[2]) / 10.0
                            if c_dist < 900.0 and abs(l_dist - r_dist) < 25.0 and abs(yaw_dps) < 2.0:
                                steer_norm = 30.0 / 45.0
                            if abs(steer_norm) < 0.05 and rng.random() > 0.10:
                                current_speed = float(max(out.left_rpm, out.right_rpm)) / 100.0 if out.actuator_enable else 0.0
                                return out
                            raw_tof = [
                                float(snapshot.tof_distance_mm[0]) + rng.normal(0, 5.0),
                                float(snapshot.tof_distance_mm[1]) + rng.normal(0, 5.0),
                                float(snapshot.tof_distance_mm[2]) + rng.normal(0, 5.0),
                            ]
                            yaw_dps = float(snapshot.gyro_dps_x10[2]) / 10.0
                            in_vec = normalize_input(
                                tof_mm=raw_tof,
                                tof_valid=[True, True, True],
                                target_heading_deg=0.0,
                                current_speed_scale=current_speed,
                                yaw_rate_dps=yaw_dps,
                            )
                            speed_norm = float(max(out.left_rpm, out.right_rpm)) / 100.0 if out.actuator_enable else 0.0
                            inputs.append(in_vec)
                            targets.append(np.array([steer_norm, speed_norm], dtype=np.float32))

                        current_speed = float(max(out.left_rpm, out.right_rpm)) / 100.0 if out.actuator_enable else 0.0
                        return out

                    run_wall_approach(
                        speed_mm_s=speed,
                        turn_rate_dps=turn_rate,
                        initial_heading_deg=init_head,
                        side_angle_deg=side_ang,
                        controller_step=step_wrapper,
                    )

    return np.array(inputs, dtype=np.float32), np.array(targets, dtype=np.float32)


def collect_expert_scenario_data(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Generate comprehensive expert state-action pairs using direct C planner evaluations."""
    inputs: list[np.ndarray] = []
    targets: list[np.ndarray] = []

    # 1. Open space navigation toward diverse headings (-45 to +45 deg)
    for _ in range(4):
        for head_deg in np.linspace(-45, 45, 31):
            for speed in [0.0, 0.3, 0.6, 1.0]:
                for yaw in [-15.0, 0.0, 15.0]:
                    in_vec = normalize_input(
                        tof_mm=[4000, 4000, 4000],
                        tof_valid=[True, True, True],
                        target_heading_deg=float(head_deg),
                        current_speed_scale=speed,
                        yaw_rate_dps=yaw,
                    )
                    steer_norm = np.clip(head_deg / 45.0, -1.0, 1.0)
                    speed_norm = 1.0
                    inputs.append(in_vec)
                    targets.append(np.array([steer_norm, speed_norm], dtype=np.float32))

    # 2. Obstacle encounters with varied geometry using smooth_avoidance_plan
    distances = [260, 350, 500, 650, 800, 1200, 2000]
    for d_c in distances:
        for d_l in distances:
            for d_r in distances:
                for head_deg in [-20, 0, 20]:
                    inp = SmoothAvoidanceInput(
                        tof_distance_mm=(ctypes.c_float * 3)(float(d_l), float(d_c), float(d_r)),
                        tof_valid=(ctypes.c_int32 * 3)(1, 1, 1),
                        target_heading_deg=float(head_deg),
                        current_steering_deg=0.0,
                        current_speed_scale=0.8,
                        dt_sec=1.0,
                    )
                    out = smooth_avoidance_plan_step(inp)
                    steer_norm = np.clip(out.steering_deg / 45.0, -1.0, 1.0)
                    speed_norm = np.clip(out.speed_scale, 0.0, 1.0)
                    for yaw in [-15.0, 0.0, 15.0]:
                        in_vec = normalize_input(
                            tof_mm=[d_l, d_c, d_r],
                            tof_valid=[True, True, True],
                            target_heading_deg=float(head_deg),
                            current_speed_scale=speed_norm,
                            yaw_rate_dps=yaw,
                        )
                        inputs.append(in_vec)
                        targets.append(np.array([steer_norm, speed_norm], dtype=np.float32))

    # 3. Invalid / missing sensor channels
    for mask in [(False, True, True), (True, False, True), (True, True, False)]:
        for head_deg in [-15, 0, 15]:
            inp = SmoothAvoidanceInput(
                tof_distance_mm=(ctypes.c_float * 3)(1000.0, 1000.0, 1000.0),
                tof_valid=(ctypes.c_int32 * 3)(int(mask[0]), int(mask[1]), int(mask[2])),
                target_heading_deg=float(head_deg),
                current_steering_deg=0.0,
                current_speed_scale=0.8,
                dt_sec=1.0,
            )
            out = smooth_avoidance_plan_step(inp)
            steer_norm = np.clip(out.steering_deg / 45.0, -1.0, 1.0)
            speed_norm = np.clip(out.speed_scale, 0.0, 1.0)
            in_vec = normalize_input(
                tof_mm=[1000, 1000, 1000],
                tof_valid=mask,
                target_heading_deg=float(head_deg),
                current_speed_scale=0.8,
                yaw_rate_dps=0.0,
            )
            inputs.append(in_vec)
            targets.append(np.array([steer_norm, speed_norm], dtype=np.float32))

    return np.array(inputs, dtype=np.float32), np.array(targets, dtype=np.float32)


def collect_fixture_telemetry() -> tuple[np.ndarray, np.ndarray]:
    """Blend real rover telemetry from fixture jsonl file."""
    inputs: list[np.ndarray] = []
    targets: list[np.ndarray] = []

    if not FIXTURE_PATH.exists():
        return np.empty((0, 10), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    with open(FIXTURE_PATH, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            sensors = entry.get("sensors", {})
            tof_mm = sensors.get("tof_mm", [1000, 1000, 1000])
            valid_flags = sensors.get("valid_flags", 15)
            gyro = sensors.get("gyro_dps_x10", [0, 0, 0])
            think = entry.get("think", {})
            steering_deg = think.get("steering_deg", 0)
            command = entry.get("command", {})
            left_rpm = command.get("left_rpm", 100)
            right_rpm = command.get("right_rpm", 100)

            valid = [bool(valid_flags & 1), bool(valid_flags & 2), bool(valid_flags & 4)]
            yaw_dps = gyro[2] / 10.0 if len(gyro) > 2 else 0.0

            avg_rpm = (abs(left_rpm) + abs(right_rpm)) / 2.0
            speed_scale = np.clip(avg_rpm / 120.0, 0.0, 1.0)
            steer_norm = np.clip(steering_deg / 45.0, -1.0, 1.0)

            in_vec = normalize_input(
                tof_mm=tof_mm,
                tof_valid=valid,
                target_heading_deg=0.0,
                current_speed_scale=speed_scale,
                yaw_rate_dps=yaw_dps,
            )
            inputs.append(in_vec)
            targets.append(np.array([steer_norm, speed_scale], dtype=np.float32))

    return np.array(inputs, dtype=np.float32), np.array(targets, dtype=np.float32)


def build_and_train_model(x_train: np.ndarray, y_train: np.ndarray) -> tf.keras.Model:
    """Build and train 10 -> 16(ReLU) -> 2 MLP model."""
    tf.keras.utils.set_random_seed(20260920)

    model = tf.keras.Sequential([
        tf.keras.layers.Input(batch_shape=(1, 10)),
        tf.keras.layers.Dense(16, activation="relu"),
        tf.keras.layers.Dense(2),
    ])

    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.005),
        loss="mse",
        metrics=["mae"],
    )

    print(f"Training MLP on {len(x_train)} samples for 60 epochs...")
    model.fit(
        x_train,
        y_train,
        batch_size=32,
        epochs=60,
        verbose=0,
        shuffle=True,
    )
    loss, mae = model.evaluate(x_train, y_train, verbose=0, batch_size=len(x_train))
    print(f"Training completed: MSE Loss = {loss:.5f}, MAE = {mae:.5f}")
    return model


def quantize_to_int8(model: tf.keras.Model, calibration_data: np.ndarray) -> bytes:
    """Convert model to full int8 quantization using representative dataset."""
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    def representative_dataset():
        for i in range(min(len(calibration_data), 300)):
            yield [calibration_data[i : i + 1].astype(np.float32)]

    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8

    tflite_model = converter.convert()
    return tflite_model


def export_c_header(tflite_bytes: bytes, output_path: Path) -> None:
    """Generate C header file containing 16-byte aligned model array and quantization metadata."""
    interpreter = tf.lite.Interpreter(model_content=tflite_bytes)
    interpreter.allocate_tensors()
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()[0]

    in_scale, in_zp = inp["quantization"]
    out_scale, out_zp = out["quantization"]
    model_len = len(tflite_bytes)

    lines: list[str] = [
        "/** =================================================================*",
        " * @file   control_mlp_model.h",
        " * @brief  完全 int8 量子化障害物回避制御 MLP モデルバイナリ定数",
        " * @note   自動生成: train_control_mlp.py (Policy Distillation)",
        " * ================================================================= */",
        "#ifndef SEROV_CPU0_CONTROL_MLP_MODEL_H",
        "#define SEROV_CPU0_CONTROL_MLP_MODEL_H",
        "",
        '#include <tk/tkernel.h>                                     /* μT-Kernel型定義 */',
        "",
        f"#define CONTROL_MLP_MODEL_BYTES            ({model_len}U)",
        f"#define CONTROL_MLP_INPUT_DIMENSION        (10U)",
        f"#define CONTROL_MLP_OUTPUT_DIMENSION       (2U)",
        f"#define CONTROL_MLP_INPUT_SCALE            ({in_scale:.8f}f)",
        f"#define CONTROL_MLP_INPUT_ZERO_POINT       ({in_zp})",
        f"#define CONTROL_MLP_OUTPUT_SCALE           ({out_scale:.8f}f)",
        f"#define CONTROL_MLP_OUTPUT_ZERO_POINT      ({out_zp})",
        "",
        "/**< 16バイト境界アライメントされたTFLM制御MLPモデルデータ */",
        "__attribute__((aligned(16)))",
        f"static const UB g_control_mlp_model[CONTROL_MLP_MODEL_BYTES] = {{",
    ]

    # Format hex bytes (12 bytes per line)
    chunk_size = 12
    for i in range(0, model_len, chunk_size):
        chunk = tflite_bytes[i : i + chunk_size]
        hex_str = ", ".join(f"0x{b:02x}U" for b in chunk)
        comma = "," if (i + chunk_size) < model_len else ""
        lines.append(f"    {hex_str}{comma}")

    lines.extend([
        "};",
        "",
        f"static const UW g_control_mlp_model_len = CONTROL_MLP_MODEL_BYTES;",
        "",
        "#endif /* SEROV_CPU0_CONTROL_MLP_MODEL_H */",
        "",
    ])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Exported C header to: {output_path} ({model_len} bytes)")


def main():
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(20260920)

    print("Collecting training data from simulator rollouts...")
    x_sim, y_sim = collect_simulator_rollouts(rng)
    print(f"Simulator rollout samples: {len(x_sim)}")

    print("Collecting expert scenario samples...")
    x_exp, y_exp = collect_expert_scenario_data(rng)
    print(f"Expert scenario samples: {len(x_exp)}")

    print("Blending real-world telemetry fixtures...")
    x_fix, y_fix = collect_fixture_telemetry()
    print(f"Fixture telemetry samples: {len(x_fix)}")

    # Combine datasets
    x_all = np.vstack([x_sim, x_exp, x_fix])
    y_all = np.vstack([y_sim, y_exp, y_fix])
    print(f"Total dataset size: {len(x_all)} samples, input shape {x_all.shape}, target shape {y_all.shape}")

    # Train model
    model = build_and_train_model(x_all, y_all)

    # Quantize to int8
    print("Quantizing model to int8 TFLite flatbuffer...")
    tflite_bytes = quantize_to_int8(model, x_all)
    TFLITE_OUTPUT_PATH.write_bytes(tflite_bytes)
    print(f"Saved int8 tflite model to: {TFLITE_OUTPUT_PATH} ({len(tflite_bytes)} bytes)")

    # Export C header
    export_c_header(tflite_bytes, HEADER_OUTPUT_PATH)
    print("Training and code generation successfully completed!")


if __name__ == "__main__":
    main()
