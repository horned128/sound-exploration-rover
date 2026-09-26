"""Load the production acoustic TFLM wrapper with the exported model on host.

Uses the repository's already built upstream TFLM object files, but compiles
the actual new firmware acoustic_tflm_runtime.cc instead of a mock shim.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "firmware/ra8p1/common/tflm"
CPU0 = ROOT / "firmware/ra8p1/SoundExplorationRover_CPU0/src"
EXISTING = ROOT / "software/acoustic-trainer/build/objects/firmware/ra8p1/common/tflm"


def inspect(model: Path, output: Path, *, add_missing_ops: bool = False, arena_bytes: int = 65536,
            input_wav: Path | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    objects = sorted(EXISTING.rglob("*.o"))
    if len(objects) < 100:
        raise RuntimeError("build upstream TFLM objects first via software/acoustic-trainer/tests/build_runtime.py")
    obj = output / "acoustic_tflm_runtime.o"
    source = CPU0 / "ai/acoustic_tflm_runtime.cc"
    if add_missing_ops or arena_bytes != 65536:
        # Only edit a diagnostic copy; never patch the user's running firmware
        # until the actual TFLM model and RAM budget have been validated.
        code = source.read_text(encoding="utf-8")
        if add_missing_ops:
            code = code.replace("MicroMutableOpResolver<10>", "MicroMutableOpResolver<16>")
            code = code.replace("(void) s_acoustic_resolver->AddReshape();",
                "(void) s_acoustic_resolver->AddReshape();\n"
                "    (void) s_acoustic_resolver->AddDequantize();\n"
                "    (void) s_acoustic_resolver->AddSquare();\n"
                "    (void) s_acoustic_resolver->AddQuantize();\n"
                "    (void) s_acoustic_resolver->AddSum();\n"
                "    (void) s_acoustic_resolver->AddRsqrt();\n"
                "    (void) s_acoustic_resolver->AddMinimum();\n"
                "    (void) s_acoustic_resolver->AddMul();")
        code = code.replace("s_acoustic_arena[ACOUSTIC_TFLM_ARENA_BYTES]",
                            f"s_acoustic_arena[{arena_bytes}U]")
        source = output / "acoustic_tflm_runtime_experiment.cc"
        source.write_text(code, encoding="utf-8")
    includes = [VENDOR, VENDOR / "third_party/flatbuffers/include", VENDOR / "third_party/gemmlowp",
                VENDOR / "third_party/ruy", VENDOR / "third_party/kissfft", CPU0,
                ROOT / "software/control-sim/shim"]
    command = ["c++", "-std=c++17", "-O2", "-fPIC", "-fno-exceptions", "-fno-rtti",
               "-fno-threadsafe-statics", "-DTF_LITE_STATIC_MEMORY", "-DTF_LITE_DISABLE_X86_NEON",
               "-DTF_LITE_STRIP_ERROR_STRINGS", "-DNDEBUG", *(f"-I{path}" for path in includes),
               "-c", str(source), "-o", str(obj)]
    subprocess.run(command, check=True)
    libpath = output / "libacoustic_tflm.dylib"
    subprocess.run(["c++", "-shared", *map(str, objects), str(obj), "-o", str(libpath)], check=True)
    lib = ctypes.CDLL(str(libpath))
    lib.acoustic_tflm_runtime_init.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
    lib.acoustic_tflm_runtime_init.restype = ctypes.c_int
    lib.acoustic_tflm_runtime_invoke.argtypes = [ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32,
                                                 ctypes.POINTER(ctypes.c_float), ctypes.c_uint32]
    lib.acoustic_tflm_runtime_invoke.restype = ctypes.c_int
    lib.acoustic_tflm_runtime_reset.argtypes = []
    raw = model.read_bytes()
    header = CPU0 / "ai/acoustic_embedding_model.h"
    source_header = header.read_text(encoding="utf-8")
    blob_start = source_header.index("= {")
    blob_end = source_header.index("};", blob_start)
    embedded_bytes = bytes(int(word, 16) for word in
                           re.findall(r"0x([0-9a-fA-F]{2})U", source_header[blob_start:blob_end]))
    storage = ctypes.create_string_buffer(len(raw) + 16)
    aligned = (ctypes.addressof(storage) + 15) & ~15
    ctypes.memmove(aligned, raw, len(raw))
    data = (ctypes.c_uint8 * len(raw)).from_address(aligned)
    rc = lib.acoustic_tflm_runtime_init(data, len(raw))
    report = {"model_bytes": len(raw), "c_header_model_matches_tflite": embedded_bytes == raw,
              "c_header_model_bytes": len(embedded_bytes),
              "upstream_objects": len(objects), "actual_tflm_init_code": rc,
              "extra_ops_experiment": add_missing_ops, "arena_bytes": arena_bytes,
              "return_codes": {"0": "OK", "-1": "invalid argument", "-2": "bad model",
                               "-3": "unsupported operator or arena allocation failure"}}
    if rc == 0:
        pcm = (ctypes.c_int8 * (80 * 32))()
        if input_wav is not None:
            import sys
            import wave
            import numpy as np
            sys.path.insert(0, str(ROOT / "software/audio_ml"))
            from short_window_probe import frontend_mel
            with wave.open(str(input_wav), "rb") as source:
                if (source.getframerate(), source.getnchannels(), source.getsampwidth()) != (16000, 2, 2):
                    raise ValueError("expected 16kHz stereo 16-bit field WAV")
                wave_data = np.frombuffer(source.readframes(source.getnframes()), dtype="<i2")
            right = wave_data.reshape(-1, 2)[:, 1][:13040].copy()
            mel = frontend_mel(right).astype(np.int8)
            if mel.shape != (80, 32):
                raise ValueError("WAV too short for 80 frontend frames")
            pcm = (ctypes.c_int8 * (80 * 32))(*mel.ravel())
        embedded = (ctypes.c_float * 64)()
        report["invoke_code"] = lib.acoustic_tflm_runtime_invoke(pcm, 80 * 32, embedded, 64)
        report["output_norm"] = sum(v*v for v in embedded) ** .5
        if input_wav is not None and report["invoke_code"] == 0:
            import tensorflow as tf
            reference = tf.lite.Interpreter(model_path=str(model))
            reference.allocate_tensors()
            reference.set_tensor(reference.get_input_details()[0]["index"], mel[None, ..., None])
            reference.invoke()
            expected = reference.get_tensor(reference.get_output_details()[0]["index"])[0]
            from numpy import array, linalg, max as array_max, abs as array_abs, dot
            actual = array(embedded[:])
            report["reference_max_abs_difference"] = float(array_max(array_abs(actual - expected)))
            report["reference_cosine_distance"] = float(1 - dot(actual, expected) /
                (max(1e-9, linalg.norm(actual)) * max(1e-9, linalg.norm(expected))))
    lib.acoustic_tflm_runtime_reset()
    (output / "host_runtime.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--add-missing-ops", action="store_true")
    parser.add_argument("--arena-bytes", type=int, default=65536)
    parser.add_argument("--input-wav", type=Path)
    args = parser.parse_args()
    inspect(args.model, args.output, add_missing_ops=args.add_missing_ops,
            arena_bytes=args.arena_bytes, input_wav=args.input_wav)
