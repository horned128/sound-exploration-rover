"""Quantize a small control MLP and compare the real C wrapper with TFLite."""
import ctypes as ct
import json
import os
from pathlib import Path
import platform

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import numpy as np
import tensorflow as tf

BUILD = Path(__file__).resolve().parents[1] / "build"


class Info(ct.Structure):
    _fields_ = [("input_bytes", ct.c_uint32), ("output_bytes", ct.c_uint32),
                ("arena_used_bytes", ct.c_uint32), ("input_scale", ct.c_float),
                ("output_scale", ct.c_float), ("input_zero_point", ct.c_int32),
                ("output_zero_point", ct.c_int32)]


def aligned_buffer(data):
    storage = ct.create_string_buffer(len(data) + 15)
    address = (ct.addressof(storage) + 15) & ~15
    ct.memmove(address, data, len(data))
    return storage, ct.c_void_p(address)


def main():
    BUILD.mkdir(exist_ok=True)
    tf.keras.utils.set_random_seed(20260911)
    rng = np.random.default_rng(20260911)
    model = tf.keras.Sequential([
        tf.keras.layers.Input(batch_shape=(1, 10)),
        tf.keras.layers.Dense(16, activation="relu"),
        tf.keras.layers.Dense(2),
    ])
    model.compile(optimizer="sgd", loss="mse")
    calibration = rng.uniform(-1, 1, (20, 10)).astype(np.float32)
    loss = float(model.train_on_batch(calibration[:1], np.zeros((1, 2), np.float32)))
    assert np.isfinite(loss)
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = lambda: ([sample[None]] for sample in calibration)
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    data = converter.convert()
    (BUILD / "smoke_int8.tflite").write_bytes(data)
    # Preserve a float model to prove the wrapper rejects incompatible tensor types.
    float_data = tf.lite.TFLiteConverter.from_keras_model(model).convert()
    reference = tf.lite.Interpreter(model_content=data,
                                    experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_REF)
    reference.allocate_tensors()
    inp, out = reference.get_input_details()[0], reference.get_output_details()[0]
    assert inp["dtype"] == out["dtype"] == np.int8
    lib = ct.CDLL(str(BUILD / ("libtflm_runtime.dylib" if platform.system() == "Darwin" else "libtflm_runtime.so")))
    lib.tflm_runtime_init.argtypes = [ct.c_void_p, ct.c_uint32]
    lib.tflm_runtime_init.restype = ct.c_int32
    lib.tflm_runtime_invoke.argtypes = [ct.c_void_p, ct.c_uint32, ct.c_void_p, ct.c_uint32]
    lib.tflm_runtime_invoke.restype = ct.c_int32
    lib.tflm_runtime_get_info.argtypes = [ct.POINTER(Info)]
    lib.tflm_runtime_get_info.restype = ct.c_int32
    lib.tflm_runtime_reset.argtypes = []
    lib.tflm_runtime_reset.restype = None
    info = Info()
    assert lib.tflm_runtime_get_info(ct.byref(info)) == -4
    assert lib.tflm_runtime_get_info(None) == -1
    assert lib.tflm_runtime_invoke(None, 0, None, 0) == -4
    assert lib.tflm_runtime_init(None, 0) == -1
    bad_storage, bad = aligned_buffer(bytes(32))
    assert lib.tflm_runtime_init(bad, 32) == -2
    storage, address = aligned_buffer(data)
    assert lib.tflm_runtime_init(ct.c_void_p(address.value + 1), len(data) - 1) == -1
    assert lib.tflm_runtime_init(address, len(data) // 2) == -2
    float_storage, float_address = aligned_buffer(float_data)
    assert lib.tflm_runtime_init(float_address, len(float_data)) == -2
    # A 128 KiB output tensor must exceed the static 96 KiB arena.
    large = tf.keras.Sequential([
        tf.keras.layers.Input(batch_shape=(1, 1)),
        tf.keras.layers.Dense(131072, use_bias=False),
    ])
    large.set_weights([np.full((1, 131072), 0.5, np.float32)])
    large_converter = tf.lite.TFLiteConverter.from_keras_model(large)
    large_converter.optimizations = [tf.lite.Optimize.DEFAULT]
    large_converter.representative_dataset = lambda: (
        [np.full((1, 1), value, np.float32)] for value in (-1, 1))
    large_converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    large_converter.inference_input_type = tf.int8
    large_converter.inference_output_type = tf.int8
    large_data = large_converter.convert()
    large_storage, large_address = aligned_buffer(large_data)
    assert lib.tflm_runtime_init(large_address, len(large_data)) == -3
    assert lib.tflm_runtime_invoke(None, 0, None, 0) == -4
    assert lib.tflm_runtime_init(address, len(data)) == 0
    assert lib.tflm_runtime_get_info(ct.byref(info)) == 0
    assert (info.input_bytes, info.output_bytes) == (10, 2)
    assert 0 < info.arena_used_bytes <= 96 * 1024
    assert (info.input_scale, info.input_zero_point) == inp["quantization"]
    assert (info.output_scale, info.output_zero_point) == out["quantization"]
    output = np.full((1, 2), 42, dtype=np.int8)
    assert lib.tflm_runtime_invoke(None, 10, output.ctypes.data, 2) == -1
    assert lib.tflm_runtime_invoke(address, 9, output.ctypes.data, 2) == -1
    assert lib.tflm_runtime_invoke(address, 10, output.ctypes.data, 1) == -1
    assert np.all(output == 42)
    cases = [np.full((1, 10), value, np.int8) for value in (-128, 0, 127)]
    cases += [rng.integers(-128, 128, (1, 10), dtype=np.int8) for _ in range(8)]
    max_error = 0
    for sample in cases:
        reference.set_tensor(inp["index"], sample)
        reference.invoke()
        expected = reference.get_tensor(out["index"])
        assert lib.tflm_runtime_invoke(sample.ctypes.data, sample.nbytes, output.ctypes.data, output.nbytes) == 0
        error = int(np.max(np.abs(output.astype(np.int16) - expected.astype(np.int16))))
        max_error = max(max_error, error)
        assert error <= 1, (error, output, expected)
    # Failed replacement invalidates the old model, then repeated init/reset must recover.
    assert lib.tflm_runtime_init(bad, 32) == -2
    assert lib.tflm_runtime_invoke(cases[0].ctypes.data, 10, output.ctypes.data, 2) == -4
    for _ in range(3):
        assert lib.tflm_runtime_init(address, len(data)) == 0
        assert lib.tflm_runtime_invoke(cases[0].ctypes.data, 10, output.ctypes.data, 2) == 0
        lib.tflm_runtime_reset()
    assert lib.tflm_runtime_get_info(ct.byref(Info())) == -4
    report = {"tensorflow": tf.__version__, "numpy": np.__version__,
              "training_loss": loss, "model_bytes": len(data), "arena_used_bytes_host": info.arena_used_bytes,
              "cases": len(cases), "max_int8_error": max_error,
              "checks": "training, full int8 conversion, C ABI inference, invalid model/size/state, arena exhaustion, reset"}
    (BUILD / "smoke_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
