"""Small target-independent query-conditioned detector trained on PCM mixtures.

The label is waveform-source inclusion, not a sound category. ESC-50 classes
listed as train_classes are the only offline training sources; validation/test
and the rover TARGET are never used for gradient updates.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import tensorflow as tf

from train_fewshot import CONTEXT, SEED, fast_frontend, mix, clips_and_positions


def pairs(manifest: Path, count: int, seed: int, class_filter: str = "train_classes",
          frontend: str = "centered") -> tuple[np.ndarray, np.ndarray]:
    config = json.loads(manifest.read_text(encoding="utf-8"))
    audio, positions = clips_and_positions(manifest)
    groups = {}
    for row in config["clips"]:
        if row["class"] in config[class_filter]:
            groups.setdefault(row["class"], []).append(row["file"])
    classes = sorted(groups)
    rng = np.random.default_rng(seed)

    def sample(category: str, exclude: str | None = None) -> tuple[str, np.ndarray]:
        choices = [f for f in groups[category] if f != exclude]
        filename = choices[int(rng.integers(len(choices)))]
        starts = positions[filename]
        first = int(starts[int(rng.integers(len(starts)))])
        return filename, audio[filename][first:first + CONTEXT]

    inputs = np.empty((count, 40, 32, 2), dtype=np.float32)
    labels = np.empty((count, 1), dtype=np.float32)
    for n in range(count):
        category = classes[int(rng.integers(len(classes)))]
        source, query = sample(category)
        other_class = classes[int(rng.integers(len(classes) - 1))]
        if other_class == category:
            other_class = classes[-1]
        _, noise = sample(other_class)
        if n & 1:
            _, candidate = sample(category if rng.random() < .5 else other_class, exclude=source)
            candidate = mix(candidate, noise, float(rng.uniform(-8, 12)))
            labels[n, 0] = 0.0
        else:
            offset = int(rng.integers(-480, 481))
            candidate = np.roll(query, offset).copy()
            if offset < 0:
                candidate[offset:] = 0
            elif offset > 0:
                candidate[:offset] = 0
            candidate = mix(candidate, noise, float(rng.uniform(-8, 12)))
            labels[n, 0] = 1.0
        if frontend == "absolute":
            query = np.clip(query.astype(np.float32) * float(10**(rng.uniform(-9, 9) / 20)),
                            -32768, 32767).astype(np.int16)
            candidate = np.clip(candidate.astype(np.float32) * float(10**(rng.uniform(-9, 9) / 20)),
                                -32768, 32767).astype(np.int16)
        inputs[n, :, :, 0] = fast_frontend(query, frontend).astype(np.float32) / 128
        inputs[n, :, :, 1] = fast_frontend(candidate, frontend).astype(np.float32) / 128
    order = rng.permutation(count)
    return inputs[order], labels[order]


def model(channels: int) -> tf.keras.Model:
    inp = tf.keras.Input((40, 32, 2), name="query_and_audio")
    x = tf.keras.layers.Conv2D(channels, 3, strides=2, padding="same", activation="relu")(inp)
    x = tf.keras.layers.DepthwiseConv2D(3, padding="same", activation="relu")(x)
    x = tf.keras.layers.Conv2D(channels * 2, 1, activation="relu")(x)
    x = tf.keras.layers.MaxPooling2D(pool_size=2)(x)
    x = tf.keras.layers.AveragePooling2D(pool_size=(10, 1))(x)
    x = tf.keras.layers.Flatten()(x)
    x = tf.keras.layers.Dense(32, activation="relu")(x)
    out = tf.keras.layers.Dense(1, activation="sigmoid", name="target_present")(x)
    return tf.keras.Model(inp, out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--pairs", type=int, default=8000)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--frontend", choices=("centered", "absolute"), default="centered")
    parser.add_argument("--channels", type=int, nargs="+", default=[8, 12],
                        help="bounded model-size sweep")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    tf.random.set_seed(SEED)
    x, y = pairs(args.manifest, args.pairs, SEED, frontend=args.frontend)
    stats = []
    for channels in args.channels:
        network = model(channels)
        network.compile(optimizer=tf.keras.optimizers.Adam(1e-3), loss="binary_crossentropy")
        history = network.fit(x, y, epochs=args.epochs, batch_size=64, verbose=0,
                              validation_split=.15, shuffle=True)
        name = f"query_{'abs_' if args.frontend == 'absolute' else ''}{channels}.keras"
        network.save(args.output / name)
        item = {"file": name, "parameters": network.count_params(),
                "loss": [float(v) for v in history.history["loss"]],
                "validation_loss": [float(v) for v in history.history["val_loss"]]}
        stats.append(item)
        print(json.dumps({"model": name, "parameters": item["parameters"],
                          "first_loss": item["loss"][0], "last_loss": item["loss"][-1],
                          "validation_loss": item["validation_loss"][-1]}))
    (args.output / "training.json").write_text(json.dumps({"source": json.loads(args.manifest.read_text())["source"],
        "pairs": args.pairs, "epochs": args.epochs, "frontend": args.frontend,
        "training_classes": json.loads(args.manifest.read_text())["train_classes"],
        "models": stats}, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
