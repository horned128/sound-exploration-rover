"""TARGET-independent, PCM-mixture metric pretraining on ESC-10.

ESC classes 0..5 are used for training; 6..7 for architecture/threshold tuning;
8..9 plus the rover's field TARGET remain unseen for final evaluation. The
on-device operation registers only five embeddings, never retrains a target CNN.
"""
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import tensorflow as tf

from esc10_fewshot_eval import read_resampled
from features import mel_filterbank, periodic_hann

WINDOW = 40
HOP = 160
CONTEXT = 400 + HOP * (WINDOW - 1)
SEED = 20260925


def fast_frontend(audio: np.ndarray, mode: str = "centered") -> np.ndarray:
    """400/160/512 HTK int8 mel; optionally retain absolute log energy."""
    if len(audio) < 400:
        return np.empty((0, 32), dtype=np.int8)
    chunks = np.lib.stride_tricks.sliding_window_view(audio.astype(np.float32) / 32768.0, 400)[::160]
    fft = np.fft.rfft(chunks * periodic_hann(np.dtype(np.float32))[None, :], n=512, axis=1)
    power = fft.real**2 + fft.imag**2
    log = np.log(np.maximum(power @ mel_filterbank().T, 1e-12))
    if mode == "centered":
        normalized = (log - log.mean(axis=1, keepdims=True)) / 0.125
    elif mode == "absolute":
        # log energy -24..+8 maps to int8 -128..+127. This preserves energy
        # without assuming any particular target frequency or loudness.
        normalized = (log + 8.0) / 0.125
    else:
        raise ValueError(f"unsupported frontend mode: {mode}")
    return np.clip(np.where(normalized >= 0, np.floor(normalized + .5),
                            np.ceil(normalized - .5)), -128, 127).astype(np.int8)


def feature(audio: np.ndarray) -> np.ndarray:
    frames = fast_frontend(audio)
    if len(frames) != WINDOW:
        raise ValueError(f"expected {WINDOW} frames, got {len(frames)}")
    return frames.astype(np.float32)[..., None] / 128.0


def clips_and_positions(manifest: Path) -> tuple[dict, dict]:
    data = json.loads(manifest.read_text(encoding="utf-8"))
    clips = {}
    positions = {}
    for row in data["clips"]:
        pcm = read_resampled(manifest.parent / row["file"])
        clips[row["file"]] = pcm
        starts = np.arange(0, len(pcm) - CONTEXT + 1, 1600)
        levels = np.array([np.sqrt(np.mean(pcm[s:s+CONTEXT].astype(np.float64)**2)) for s in starts])
        if len(starts) == 0:
            raise ValueError(row["file"])
        positions[row["file"]] = starts[levels >= np.percentile(levels, 55)]
    return clips, positions


def mix(target: np.ndarray, background: np.ndarray, snr_db: float) -> np.ndarray:
    t = target.astype(np.float32)
    b = background.astype(np.float32)
    tr = np.sqrt(np.mean(t * t))
    br = np.sqrt(np.mean(b * b))
    result = t + b * (tr / max(1.0, br)) * (10.0 ** (-snr_db / 20.0))
    result *= min(1.0, 30000.0 / max(30000.0, np.max(np.abs(result))))
    return np.clip(np.rint(result), -32768, 32767).astype(np.int16)


def training_data(manifest: Path, count: int, rng: np.random.Generator,
                  task: str = "category") -> tuple[np.ndarray, ...]:
    metadata = json.loads(manifest.read_text(encoding="utf-8"))
    rows = metadata["clips"]
    clips, positions = clips_and_positions(manifest)
    groups = {}
    for row in rows:
        groups.setdefault(row["class"], []).append(row["file"])
    categories = metadata.get("train_classes", sorted(groups)[:6])

    def sample(name: str, forbidden: str | None = None) -> tuple[str, np.ndarray]:
        files = [f for f in groups[name] if f != forbidden]
        file = files[int(rng.integers(len(files)))]
        starts = positions[file]
        start = int(starts[int(rng.integers(len(starts)))])
        return file, clips[file][start:start + CONTEXT]

    anchors, positives, negatives = [], [], []
    for _ in range(count):
        category = categories[int(rng.integers(len(categories)))]
        alternate = categories[int(rng.integers(len(categories) - 1))]
        if alternate == category:
            alternate = categories[-1]
        anchor_file, a = sample(category)
        _, noise = sample(alternate)
        _, other = sample(category, forbidden=anchor_file)
        if task == "source":
            # Distinct environmental event IDs, NOT category labels, are the
            # negatives. The same unknown source remains present in its mix.
            shifted = int(rng.integers(-480, 481))
            p = np.roll(a, shifted).copy()
            if shifted > 0:
                p[:shifted] = 0
            elif shifted < 0:
                p[shifted:] = 0
            if rng.random() < .5:
                delayed = p.astype(np.float32)
                delayed[320:] += float(rng.uniform(.02, .14)) * delayed[:-320]
                p = np.clip(delayed, -32768, 32767).astype(np.int16)
            _, n = sample(category if rng.random() < .5 else alternate,
                          forbidden=anchor_file)
            p = mix(p, noise, float(rng.uniform(-8, 12)))
            if rng.random() < .35:
                a = mix(a, other, float(rng.uniform(3, 18)))
        else:
            _, p = sample(category, forbidden=anchor_file)
            _, n = sample(alternate)
            if rng.random() < .85:
                p = mix(p, noise, float(rng.uniform(-6, 12)))
            if rng.random() < .40:
                a = mix(a, noise, float(rng.uniform(0, 18)))
        if rng.random() < .50:
            n = mix(n, other if task != "source" else noise, float(rng.uniform(0, 12)))
        anchors.append(feature(a))
        positives.append(feature(p))
        negatives.append(feature(n))
    return (np.stack(anchors).astype(np.float32), np.stack(positives).astype(np.float32),
            np.stack(negatives).astype(np.float32))


def create_model(channels: int, dimension: int, temporal_pool: bool = False) -> tf.keras.Model:
    inp = tf.keras.Input((WINDOW, 32, 1), name="logmel")
    x = tf.keras.layers.Conv2D(channels, 3, strides=2, padding="same", activation="relu")(inp)
    x = tf.keras.layers.DepthwiseConv2D(3, padding="same", activation="relu")(x)
    x = tf.keras.layers.Conv2D(channels * 2, 1, activation="relu")(x)
    x = tf.keras.layers.MaxPooling2D(pool_size=2)(x)
    if temporal_pool:
        # Pool across ten time cells, retaining mel-position information. This
        # makes +/-40ms inference-grid shifts less brittle than Flatten alone.
        x = tf.keras.layers.AveragePooling2D(pool_size=(10, 1))(x)
    x = tf.keras.layers.Flatten()(x)
    out = tf.keras.layers.Dense(dimension, name="embedding")(x)
    return tf.keras.Model(inp, out)


def train(model: tf.keras.Model, arrays: tuple[np.ndarray, ...], epochs: int) -> list[float]:
    optimizer = tf.keras.optimizers.Adam(learning_rate=1e-3)
    history = []
    a, p, n = arrays
    for epoch in range(epochs):
        order = np.random.default_rng(SEED + epoch).permutation(len(a))
        losses = []
        for indices in np.array_split(order, max(1, len(order) // 64)):
            with tf.GradientTape() as tape:
                ea = tf.math.l2_normalize(model(a[indices], training=True), axis=-1)
                ep = tf.math.l2_normalize(model(p[indices], training=True), axis=-1)
                en = tf.math.l2_normalize(model(n[indices], training=True), axis=-1)
                pos = 1 - tf.reduce_sum(ea * ep, axis=-1)
                neg = 1 - tf.reduce_sum(ea * en, axis=-1)
                loss = tf.reduce_mean(tf.nn.relu(.25 + pos - neg))
            gradients = tape.gradient(loss, model.trainable_variables)
            optimizer.apply_gradients(zip(gradients, model.trainable_variables))
            losses.append(float(loss))
        history.append(float(np.mean(losses)))
    return history


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--examples", type=int, default=1800)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--task", choices=("category", "source"), default="source")
    parser.add_argument("--temporal-pool", action="store_true")
    args = parser.parse_args()
    tf.random.set_seed(SEED)
    np.random.seed(SEED)
    random.seed(SEED)
    args.output.mkdir(parents=True, exist_ok=True)
    arrays = training_data(args.manifest, args.examples, np.random.default_rng(SEED), args.task)
    report = []
    for channels, dimension in ((8, 32), (12, 48)):
        model = create_model(channels, dimension, args.temporal_pool)
        history = train(model, arrays, args.epochs)
        path = args.output / f"metric_{'pool_' if args.temporal_pool else ''}{channels}_{dimension}.keras"
        model.save(path)
        report.append({"model": path.name, "parameters": model.count_params(), "loss": history})
        print(f"{path.name}: {history[0]:.4f} -> {history[-1]:.4f}")
    (args.output / "training.json").write_text(json.dumps({"seed": SEED,
        "class_split": "manifest train_classes (or first six ESC-10) only; validation/test and field TARGET never trained",
        "train_classes": json.loads(args.manifest.read_text(encoding="utf-8")).get("train_classes"),
        "examples": args.examples, "epochs": args.epochs, "task": args.task,
        "temporal_pool": args.temporal_pool,
        "models": report}, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
