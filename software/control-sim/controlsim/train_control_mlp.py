"""Train an int8, clearance-aware obstacle-avoidance MLP and export its C header.

The prior trainer distilled a potential-field planner.  Its teacher could reduce an
already selected avoidance turn when a side-wall measurement changed, so retraining
on that data only reproduced the same weakness.  This trainer labels synthetic and
recorded ToF states with a clearance-aware expert: a near center obstacle chooses the
wider side and keeps a minimum turn until the forward clearance is recovered.
"""

from __future__ import annotations

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


def clearance_aware_expert(
    tof_mm: list[float] | tuple[float, ...],
    tof_valid: list[bool] | tuple[bool, ...],
    target_heading_deg: float,
) -> tuple[float, float]:
    """Return a robust steering/speed label for a single ToF observation.

    A front obstacle within 900 mm selects the wider side once and imposes a
    28--42 degree minimum turn.  Importantly, attraction toward the sound source
    cannot cancel that turn.  The firmware adds the same rule as a safety guard,
    so quantization cannot weaken it at runtime.
    """
    distances = [float(tof_mm[i]) if tof_valid[i] else 4000.0 for i in range(3)]
    left_mm, center_mm, right_mm = distances
    goal_steer = float(np.clip(target_heading_deg, -45.0, 45.0))

    if center_mm < 900.0:
        if (right_mm - left_mm) > 75.0:
            direction = 1.0
        elif (left_mm - right_mm) > 75.0:
            direction = -1.0
        elif abs(goal_steer) >= 5.0:
            direction = 1.0 if goal_steer > 0.0 else -1.0
        else:
            direction = 1.0

        closeness = float(np.clip((900.0 - center_mm) / (900.0 - 150.0), 0.0, 1.0))
        minimum_steer = 28.0 + closeness * (42.0 - 28.0)
        steer_deg = direction * max(minimum_steer, direction * goal_steer)
        speed_scale = 0.60 - 0.30 * closeness
    elif (left_mm < 700.0) and ((right_mm - left_mm) > 75.0):
        closeness = float(np.clip((700.0 - left_mm) / (700.0 - 150.0), 0.0, 1.0))
        steer_deg = max(goal_steer, 14.0 + 16.0 * closeness)
        speed_scale = 0.85 - 0.25 * closeness
    elif (right_mm < 700.0) and ((left_mm - right_mm) > 75.0):
        closeness = float(np.clip((700.0 - right_mm) / (700.0 - 150.0), 0.0, 1.0))
        steer_deg = min(goal_steer, -(14.0 + 16.0 * closeness))
        speed_scale = 0.85 - 0.25 * closeness
    else:
        steer_deg = goal_steer
        speed_scale = 1.0

    return float(np.clip(steer_deg, -45.0, 45.0)), float(np.clip(speed_scale, 0.25, 1.0))


def collect_clearance_expert_data(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Generate balanced, noisy ToF geometries and label them with the new expert."""
    inputs: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    distances = np.array([250, 320, 420, 520, 620, 700, 760, 840, 920, 1100, 1500, 2200, 4000])
    headings = [-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0]

    # Dense coverage around the 700--900 mm handoff prevents a weak-turn band.
    for left_mm in distances:
        for center_mm in distances:
            for right_mm in distances:
                for heading_deg in headings:
                    steer_deg, speed_scale = clearance_aware_expert(
                        [float(left_mm), float(center_mm), float(right_mm)], [True, True, True], heading_deg
                    )
                    for yaw_dps in (-20.0, 0.0, 20.0):
                        current_speed = float(rng.choice([0.0, 0.35, 0.7, 1.0]))
                        noisy_tof = [
                            float(np.clip(left_mm + rng.normal(0.0, 8.0), 150.0, 4000.0)),
                            float(np.clip(center_mm + rng.normal(0.0, 8.0), 150.0, 4000.0)),
                            float(np.clip(right_mm + rng.normal(0.0, 8.0), 150.0, 4000.0)),
                        ]
                        inputs.append(normalize_input(noisy_tof, [True, True, True], heading_deg,
                                                      current_speed, yaw_dps))
                        targets.append(np.array([steer_deg / 45.0, speed_scale], dtype=np.float32))

    # Missing channels are learned as conservative open-space substitutions; the
    # firmware's sensor-liveness layer still vetoes motion when safety requires it.
    for mask in [(False, True, True), (True, False, True), (True, True, False)]:
        for heading_deg in headings:
            steer_deg, speed_scale = clearance_aware_expert([1000.0, 1000.0, 1000.0], list(mask), heading_deg)
            inputs.append(normalize_input([1000.0, 1000.0, 1000.0], list(mask), heading_deg, 0.5, 0.0))
            targets.append(np.array([steer_deg / 45.0, speed_scale], dtype=np.float32))

    return np.array(inputs, dtype=np.float32), np.array(targets, dtype=np.float32)


def collect_fixture_telemetry() -> tuple[np.ndarray, np.ndarray]:
    """Use real ToF distributions while relabeling them with the safety expert.

    The old fixture contains commands from the policy being replaced; using those
    commands as labels would teach the old weak response back into the new model.
    """
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
            command = entry.get("command", {})
            left_rpm = command.get("left_rpm", 100)
            right_rpm = command.get("right_rpm", 100)

            valid = [bool(valid_flags & 1), bool(valid_flags & 2), bool(valid_flags & 4)]
            yaw_dps = gyro[2] / 10.0 if len(gyro) > 2 else 0.0

            avg_rpm = (abs(left_rpm) + abs(right_rpm)) / 2.0
            speed_scale = np.clip(avg_rpm / 120.0, 0.0, 1.0)
            steering_deg, target_speed = clearance_aware_expert(
                [float(value) for value in tof_mm], valid, target_heading_deg=0.0
            )

            in_vec = normalize_input(
                tof_mm=tof_mm,
                tof_valid=valid,
                target_heading_deg=0.0,
                current_speed_scale=speed_scale,
                yaw_rate_dps=yaw_dps,
            )
            inputs.append(in_vec)
            targets.append(np.array([steering_deg / 45.0, target_speed], dtype=np.float32))

    return np.array(inputs, dtype=np.float32), np.array(targets, dtype=np.float32)


def build_and_train_model(x_train: np.ndarray, y_train: np.ndarray) -> tf.keras.Model:
    """Build and train a 10 -> 24(ReLU) -> 16(ReLU) -> 2 MLP model."""
    tf.keras.utils.set_random_seed(20260920)

    model = tf.keras.Sequential([
        tf.keras.layers.Input(batch_shape=(1, 10)),
        tf.keras.layers.Dense(24, activation="relu"),
        tf.keras.layers.Dense(16, activation="relu"),
        tf.keras.layers.Dense(2),
    ])

    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.005),
        loss="mse",
        metrics=["mae"],
    )

    print(f"Training clearance-aware MLP on {len(x_train)} samples for 60 epochs...")
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

    print("Collecting clearance-aware expert scenario samples...")
    x_exp, y_exp = collect_clearance_expert_data(rng)
    print(f"Clearance-aware expert samples: {len(x_exp)}")

    print("Blending real-world telemetry fixtures...")
    x_fix, y_fix = collect_fixture_telemetry()
    print(f"Fixture telemetry samples: {len(x_fix)}")

    # Combine datasets
    x_all = np.vstack([x_exp, x_fix])
    y_all = np.vstack([y_exp, y_fix])
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
