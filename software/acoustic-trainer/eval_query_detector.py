"""Class-disjoint ROC probe for query-conditioned mixture recognition.

Validation/test classes are not used for model training. Synthetic pair labels
are known exactly, while clip-scale event latency still requires hardware.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import tensorflow as tf

from train_query_detector import pairs


def predict(model: tf.keras.Model, data: np.ndarray) -> np.ndarray:
    return np.concatenate([model(batch, training=False).numpy().ravel()
                           for batch in np.array_split(data, max(1, len(data)//256))])


def evaluate(manifest: Path, models: Path, output: Path, count: int, frontend: str = "centered") -> dict:
    validation_x, validation_y = pairs(manifest, count, 1234, "validation_classes", frontend)
    test_x, test_y = pairs(manifest, count, 5678, "test_classes", frontend)
    summary = []
    for model_path in sorted(models.glob("query_*.keras")):
        model = tf.keras.models.load_model(model_path, compile=False)
        val_score = predict(model, validation_x)
        test_score = predict(model, test_x)
        def metrics(score: np.ndarray, truth: np.ndarray, threshold: float) -> dict:
            positive = truth.ravel() == 1
            fp = int(np.count_nonzero((score >= threshold) & ~positive))
            tp = int(np.count_nonzero((score >= threshold) & positive))
            return {"threshold": float(threshold), "true_positives": tp,
                    "false_positives": fp, "positives": int(positive.sum()),
                    "negatives": int((~positive).sum()),
                    "recall": round(tp / positive.sum(), 4),
                    "fpr": round(fp / (~positive).sum(), 5),
                    "precision": round(tp / (tp + fp), 4) if tp + fp else None}
        candidates = np.linspace(.5, .9999, 60)
        eligible = [t for t in candidates if metrics(val_score, validation_y, t)["false_positives"] <= 1]
        selected = min(eligible) if eligible else 1.0
        summary.append({"model": model_path.name, "parameter_count": model.count_params(),
                        "validation_selected": metrics(val_score, validation_y, selected),
                        "test_at_selected": metrics(test_score, test_y, selected),
                        "grid": [{"validation": metrics(val_score, validation_y, t),
                                  "test": metrics(test_score, test_y, t)} for t in (.5, .7, .9, .97, .99, .999)]})
    result = {"source": json.loads(manifest.read_text(encoding="utf-8"))["source"],
              "pairs_per_split": count, "uncertainty": "Sampled pairs are correlated; this is not FP/hour.",
              "models": summary}
    output.mkdir(parents=True, exist_ok=True)
    (output / "query_eval.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps([{k: v for k, v in r.items() if k != "grid"} for r in summary], indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--models", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--pairs", type=int, default=3000)
    parser.add_argument("--frontend", choices=("centered", "absolute"), default="centered")
    args = parser.parse_args()
    evaluate(args.manifest, args.models, args.output, args.pairs, args.frontend)
