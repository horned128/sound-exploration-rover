"""Convert acoustic embedding model to int8 TFLite and C header for RA8P1 Cortex-M85 (Helium)."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import tensorflow as tf

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/acoustic-trainer"))

from train_embedding import WINDOW_FRAMES, MEL_BINS


def representative_dataset_gen(model_input_shape):
    """Representative dataset using realistic log-mel range (-1.0 to 1.0)."""
    rng = np.random.default_rng(20260926)
    for _ in range(200):
        # Realistic log-mel distributed around -0.5 to 0.5
        sample = rng.normal(loc=0.0, scale=0.4, size=model_input_shape).astype(np.float32)
        sample = np.clip(sample, -1.0, 0.992)
        yield [sample]


def convert_to_tflite_and_c_header(keras_model_path: Path, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    model = tf.keras.models.load_model(str(keras_model_path), safe_mode=False)

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = lambda: representative_dataset_gen((1, WINDOW_FRAMES, MEL_BINS, 1))
    
    # Ensure full integer quantization for Helium MVE
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.float32  # Output normalized float embedding for easy cosine distance

    tflite_model = converter.convert()

    tflite_path = output_dir / "acoustic_embedding_model.tflite"
    tflite_path.write_bytes(tflite_model)
    print(f"Saved TFLite model: {tflite_path} ({len(tflite_model):,} bytes)")

    # Generate C header
    header_path = output_dir / "acoustic_embedding_model.h"
    var_name = "g_acoustic_embedding_model"

    lines = [
        "/** =================================================================*",
        " * @file   acoustic_embedding_model.h",
        " * @brief  完全 int8 量子化音響埋め込み (Acoustic Embedding) TFLM モデル定数",
        " * @note   自動生成: export_tflite.py (ESC-50 + Motor noise robust pretrained CNN)",
        " * ================================================================= */",
        "#ifndef SEROV_CPU0_ACOUSTIC_EMBEDDING_MODEL_H",
        "#define SEROV_CPU0_ACOUSTIC_EMBEDDING_MODEL_H",
        "",
        "#include <tk/tkernel.h>                                     /* μT-Kernel型定義 */",
        "",
        f"#define ACOUSTIC_EMBEDDING_MODEL_BYTES     ({len(tflite_model)}U)",
        "#define ACOUSTIC_EMBEDDING_INPUT_FRAMES    (80U)",
        "#define ACOUSTIC_EMBEDDING_INPUT_BINS      (32U)",
        "#define ACOUSTIC_EMBEDDING_DIMENSION       (64U)",
        "#define ACOUSTIC_EMBEDDING_SAMPLE_COUNT    (5U)",
        "#define ACOUSTIC_EMBEDDING_SAFE_THRESHOLD  (0.35f)",
        "",
        "/**< 16バイト境界アライメントされたTFLM音響埋め込みモデルデータ */",
        "__attribute__((aligned(16)))",
        f"static const UB {var_name}[ACOUSTIC_EMBEDDING_MODEL_BYTES] = {{",
    ]

    bytes_per_line = 12
    for i in range(0, len(tflite_model), bytes_per_line):
        chunk = tflite_model[i : i + bytes_per_line]
        hex_str = ", ".join(f"0x{b:02x}U" for b in chunk)
        lines.append(f"    {hex_str},")

    lines.extend([
        "};",
        "",
        "#endif /* SEROV_CPU0_ACOUSTIC_EMBEDDING_MODEL_H */",
        "",
    ])

    header_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Saved C header: {header_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    convert_to_tflite_and_c_header(args.model, args.output_dir)
