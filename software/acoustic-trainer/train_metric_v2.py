"""Train Metric Embedding Model v2: High Frequency Resolution + Hard Negative Mining.

Key enhancements over v1:
1. Retain frequency resolution: Conv2D stride=(2, 1) instead of (2, 2), keeping 32 mel bins.
   Max intermediate tensor: 40x32x16 = 20,480 bytes (fits within ~42-48 KiB arena).
2. Explicit Hard Negative Mining: 60% same-category alternative clips, 25% other classes, 15% pure noise (motor/tv).
3. Larger contrastive margin (0.45) with Cosine Triplet + InfoNCE temperature=0.07.
4. Clean int8 quantized graph with float32 dequantized output for host/firmware ABI compatibility.
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys
from pathlib import Path

import numpy as np
import tensorflow as tf

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/acoustic-trainer"))
sys.path.insert(0, str(ROOT / "software/audio_ml"))

from features import mel_filterbank, periodic_hann
from esc10_fewshot_eval import read_resampled

SAMPLE_RATE = 16000
WINDOW_FRAMES = 80
HOP_SAMPLES = 160
WINDOW_SAMPLES = 400
TOTAL_SAMPLES = WINDOW_SAMPLES + HOP_SAMPLES * (WINDOW_FRAMES - 1)  # 13,040 samples (~815ms)
MEL_BINS = 32
EMBEDDING_DIM = 64
SEED = 20260926


def extract_log_mel_80(audio: np.ndarray) -> np.ndarray:
    """Extract 80 frames x 32 bins log-mel matching firmware contract."""
    if len(audio) < TOTAL_SAMPLES:
        audio = np.pad(audio, (0, TOTAL_SAMPLES - len(audio)))
    elif len(audio) > TOTAL_SAMPLES:
        audio = audio[:TOTAL_SAMPLES]

    chunks = np.lib.stride_tricks.sliding_window_view(audio.astype(np.float32) / 32768.0, WINDOW_SAMPLES)[::HOP_SAMPLES]
    chunks = chunks[:WINDOW_FRAMES]
    fft = np.fft.rfft(chunks * periodic_hann(np.dtype(np.float32))[None, :], n=512, axis=1)
    power = fft.real**2 + fft.imag**2
    log = np.log(np.maximum(power @ mel_filterbank().T, 1e-12))
    normalized = (log - log.mean(axis=1, keepdims=True)) / 0.125
    mel_i8 = np.clip(np.where(normalized >= 0, np.floor(normalized + 0.5), np.ceil(normalized - 0.5)), -128, 127).astype(np.int8)
    return mel_i8.astype(np.float32) / 128.0


def mix(target: np.ndarray, background: np.ndarray, snr_db: float) -> np.ndarray:
    t = target.astype(np.float32)
    b = background.astype(np.float32)
    tr = np.sqrt(np.mean(t * t))
    br = np.sqrt(np.mean(b * b))
    result = t + b * (tr / max(1.0, br)) * (10.0 ** (-snr_db / 20.0))
    result *= min(1.0, 30000.0 / max(30000.0, np.max(np.abs(result))))
    return np.clip(np.rint(result), -32768, 32767).astype(np.int16)


def load_noise_pcm(motor_dir: Path, tv_manifest: Path) -> list[np.ndarray]:
    import wave
    noises = []
    if motor_dir.exists():
        for p in sorted(motor_dir.glob("*.wav")):
            with wave.open(str(p), "rb") as w:
                buf = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
                if w.getnchannels() == 2:
                    noises.append(buf.reshape(-1, 2)[:, 1].copy())
                else:
                    noises.append(buf.copy())
    if tv_manifest.exists():
        data = json.loads(tv_manifest.read_text(encoding="utf-8"))
        for row in data.get("clips", []):
            if "tv" in row.get("label", "") or "background" in row.get("label", ""):
                wav_file = row.get("wav")
                if wav_file:
                    p = tv_manifest.parent / wav_file
                    if p.exists():
                        noises.append(read_resampled(p))
    return noises


def build_v2_model(embedding_dim: int = EMBEDDING_DIM) -> tf.keras.Model:
    """Build a high-resolution frequency-preserving CNN (~10k params)."""
    inp = tf.keras.Input(shape=(WINDOW_FRAMES, MEL_BINS, 1), name="logmel_80x32")

    # Block 1: stride=(2, 1) keeps 32 frequency bins! Output: (40, 32, 16) = 20,480 bytes
    x = tf.keras.layers.Conv2D(16, (3, 3), strides=(2, 1), padding="same", use_bias=False)(inp)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU(max_value=6.0)(x)

    # Block 2: Depthwise Separable + MaxPool(2, 2) -> (20, 16, 24) = 7,680 bytes
    x = tf.keras.layers.DepthwiseConv2D((3, 3), padding="same", use_bias=False)(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU(max_value=6.0)(x)
    x = tf.keras.layers.Conv2D(24, (1, 1), use_bias=False)(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU(max_value=6.0)(x)
    x = tf.keras.layers.MaxPooling2D(pool_size=(2, 2))(x)

    # Block 3: Depthwise Separable + MaxPool(2, 2) -> (10, 8, 32) = 2,560 bytes
    x = tf.keras.layers.DepthwiseConv2D((3, 3), padding="same", use_bias=False)(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU(max_value=6.0)(x)
    x = tf.keras.layers.Conv2D(32, (1, 1), use_bias=False)(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU(max_value=6.0)(x)
    x = tf.keras.layers.MaxPooling2D(pool_size=(2, 2))(x)

    # Block 4: Depthwise Separable -> (10, 8, 48) = 3,840 bytes
    x = tf.keras.layers.DepthwiseConv2D((3, 3), padding="same", use_bias=False)(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU(max_value=6.0)(x)
    x = tf.keras.layers.Conv2D(48, (1, 1), use_bias=False)(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.ReLU(max_value=6.0)(x)

    # Global Average Pooling -> (48,)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)

    # Linear Projection to 64D embedding (NO UnitNormalization inside graph)
    embedding = tf.keras.layers.Dense(embedding_dim, use_bias=False, name="embedding")(x)

    return tf.keras.Model(inputs=inp, outputs=embedding, name="metric_v2_embedding")


def generate_triplets_v2(manifest_path: Path, noises: list[np.ndarray], count: int, rng: np.random.Generator):
    """Generate (anchor, positive, negative) with aggressive hard negative mining."""
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    clips_info = data["clips"]
    train_classes = set(data.get("train_classes", []))
    train_clips = [c for c in clips_info if c["class"] in train_classes]

    pcm_cache = {}
    class_groups = {}
    for c in train_clips:
        p = manifest_path.parent / c["file"]
        if p.exists():
            arr = read_resampled(p)
            pcm_cache[c["file"]] = arr
            class_groups.setdefault(c["class"], []).append(c["file"])

    classes_list = list(class_groups.keys())

    anchors, positives, negatives = [], [], []

    for _ in range(count):
        cls = classes_list[rng.integers(len(classes_list))]
        files = class_groups[cls]
        file_a = files[rng.integers(len(files))]
        raw_a = pcm_cache[file_a]

        # Extract active snippet of length TOTAL_SAMPLES
        if len(raw_a) <= TOTAL_SAMPLES:
            start_a = 0
            snippet_a = raw_a
        else:
            starts = np.arange(0, len(raw_a) - TOTAL_SAMPLES + 1, 320)
            levels = [np.sqrt(np.mean(raw_a[s:s+TOTAL_SAMPLES].astype(np.float64)**2)) for s in starts]
            high_idx = np.where(levels >= np.percentile(levels, 40))[0]
            start_a = starts[rng.choice(high_idx)] if len(high_idx) > 0 else 0
            snippet_a = raw_a[start_a:start_a + TOTAL_SAMPLES]

        # Anchor: slight time shift
        shift = int(rng.integers(-160, 161))
        a_wav = np.roll(snippet_a, shift).copy()
        if shift > 0:
            a_wav[:shift] = 0
        elif shift < 0:
            a_wav[shift:] = 0

        # Positive: same snippet + noise (motor or TV) with SNR -6 to +12 dB
        if noises and rng.random() < 0.90:
            bg = noises[rng.integers(len(noises))]
            if len(bg) > TOTAL_SAMPLES:
                bg_offset = int(rng.integers(0, len(bg) - TOTAL_SAMPLES))
                bg_snip = bg[bg_offset:bg_offset + TOTAL_SAMPLES]
            else:
                bg_snip = np.pad(bg, (0, TOTAL_SAMPLES - len(bg)))
            snr = float(rng.uniform(-6.0, 12.0))
            p_wav = mix(snippet_a, bg_snip, snr)
        else:
            p_wav = a_wav

        # Negative selection:
        # 1. 50% hard negative: other file in same class
        # 2. 30% medium negative: different class
        # 3. 20% pure noise: pure motor/tv background snippet
        neg_type = rng.random()
        if neg_type < 0.50 and len(files) > 1:
            other_files = [f for f in files if f != file_a]
            file_n = other_files[rng.integers(len(other_files))]
            raw_n = pcm_cache[file_n]
            if len(raw_n) <= TOTAL_SAMPLES:
                snippet_n = raw_n
            else:
                starts_n = np.arange(0, len(raw_n) - TOTAL_SAMPLES + 1, 320)
                start_n = int(rng.choice(starts_n))
                snippet_n = raw_n[start_n:start_n + TOTAL_SAMPLES]
            n_wav = snippet_n
        elif neg_type < 0.80 or not noises:
            other_classes = [c for c in classes_list if c != cls]
            other_cls = other_classes[rng.integers(len(other_classes))]
            other_files = class_groups[other_cls]
            file_n = other_files[rng.integers(len(other_files))]
            raw_n = pcm_cache[file_n]
            if len(raw_n) <= TOTAL_SAMPLES:
                snippet_n = raw_n
            else:
                starts_n = np.arange(0, len(raw_n) - TOTAL_SAMPLES + 1, 320)
                start_n = int(rng.choice(starts_n))
                snippet_n = raw_n[start_n:start_n + TOTAL_SAMPLES]
            n_wav = snippet_n
        else:
            # Pure noise as negative!
            bg = noises[rng.integers(len(noises))]
            if len(bg) > TOTAL_SAMPLES:
                bg_offset = int(rng.integers(0, len(bg) - TOTAL_SAMPLES))
                n_wav = bg[bg_offset:bg_offset + TOTAL_SAMPLES]
            else:
                n_wav = np.pad(bg, (0, TOTAL_SAMPLES - len(bg)))

        anchors.append(extract_log_mel_80(a_wav))
        positives.append(extract_log_mel_80(p_wav))
        negatives.append(extract_log_mel_80(n_wav))

    return (
        np.stack(anchors)[..., None].astype(np.float32),
        np.stack(positives)[..., None].astype(np.float32),
        np.stack(negatives)[..., None].astype(np.float32),
    )


def train_metric_v2(model: tf.keras.Model, triplets: tuple[np.ndarray, np.ndarray, np.ndarray], epochs: int = 15, batch_size: int = 64) -> list[float]:
    lr_schedule = tf.keras.optimizers.schedules.CosineDecay(initial_learning_rate=1e-3, decay_steps=epochs * (len(triplets[0]) // batch_size))
    optimizer = tf.keras.optimizers.Adam(learning_rate=lr_schedule)
    anchors, positives, negatives = triplets
    total = len(anchors)
    history = []

    for epoch in range(epochs):
        indices = np.random.permutation(total)
        epoch_losses = []
        for start in range(0, total, batch_size):
            batch_idx = indices[start:start + batch_size]
            a_b = anchors[batch_idx]
            p_b = positives[batch_idx]
            n_b = negatives[batch_idx]

            with tf.GradientTape() as tape:
                ea = tf.math.l2_normalize(model(a_b, training=True), axis=-1)
                ep = tf.math.l2_normalize(model(p_b, training=True), axis=-1)
                en = tf.math.l2_normalize(model(n_b, training=True), axis=-1)

                pos_dist = 1.0 - tf.reduce_sum(ea * ep, axis=-1)
                neg_dist = 1.0 - tf.reduce_sum(ea * en, axis=-1)
                margin = 0.45
                triplet_loss = tf.reduce_mean(tf.nn.relu(pos_dist - neg_dist + margin))

                # InfoNCE with temperature 0.07
                sim_pos = tf.reduce_sum(ea * ep, axis=-1) / 0.07
                sim_neg = tf.matmul(ea, en, transpose_b=True) / 0.07
                logits = tf.concat([sim_pos[:, None], sim_neg], axis=1)
                labels = tf.zeros(tf.shape(logits)[0], dtype=tf.int32)
                infonce_loss = tf.reduce_mean(tf.nn.sparse_softmax_cross_entropy_with_logits(labels=labels, logits=logits))

                loss = triplet_loss + infonce_loss

            grads = tape.gradient(loss, model.trainable_variables)
            optimizer.apply_gradients(zip(grads, model.trainable_variables))
            epoch_losses.append(float(loss))

        avg_loss = float(np.mean(epoch_losses))
        history.append(avg_loss)
        print(f"Epoch {epoch+1:2d}/{epochs:2d}: loss = {avg_loss:.4f}")

    return history


def export_clean_int8_tflite(model: tf.keras.Model, rep_data: np.ndarray, output_path: Path):
    def rep_gen():
        for i in range(min(200, len(rep_data))):
            yield [rep_data[i:i+1].astype(np.float32)]

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = rep_gen
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.float32

    tflite_model = converter.convert()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(tflite_model)
    print(f"Exported clean int8 TFLite model: {output_path} ({len(tflite_model):,} bytes)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--motor", type=Path, required=True)
    parser.add_argument("--tv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--triplets", type=int, default=7000)
    parser.add_argument("--epochs", type=int, default=15)
    args = parser.parse_args()

    np.random.seed(SEED)
    random.seed(SEED)
    tf.random.set_seed(SEED)

    print("Loading background noise PCM...")
    noises = load_noise_pcm(args.motor, args.tv)
    print(f"Loaded {len(noises)} noise clips.")

    print(f"Generating {args.triplets} training triplets...")
    rng = np.random.default_rng(SEED)
    triplets = generate_triplets_v2(args.manifest, noises, args.triplets, rng)
    print(f"Triplets ready: {triplets[0].shape}")

    print("Building compact metric v2 model...")
    model = build_v2_model(EMBEDDING_DIM)
    model.summary()

    print("Training metric v2 embedding model...")
    train_metric_v2(model, triplets, epochs=args.epochs)

    output_tflite = args.output / "metric_v2_model.tflite"
    print(f"Exporting clean int8 TFLite model to {output_tflite}...")
    export_clean_int8_tflite(model, triplets[0], output_tflite)


if __name__ == "__main__":
    main()
