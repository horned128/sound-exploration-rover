"""Phase 2の背景学習と現場照合を定義する決定論的なホスト参照実装。

このモジュールは実録音の精度を主張するものではない。CPU0へ移植する前に、背景AEの
RLS更新、能動フレーム選別、96次元要約、個別見本へのcosine距離を同じ数値契約で検証する。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

FEATURE_BINS = 32
HIDDEN_UNITS = 16
ENCODER_SEED = 0x53455231
RLS_FORGETTING_FACTOR = 0.95
RLS_INITIAL_DIAGONAL = 1.0 / 0.01
ACTIVE_FRAME_STDDEV_LSB = 12.0
MIN_ACTIVE_FRAMES = 10
SAMPLE_COUNT = 5
SUMMARY_DIMENSION = FEATURE_BINS * 3


def _lcg_uniform(seed: int, count: int) -> tuple[np.ndarray, int]:
    """Cでも再現しやすい32-bit LCGから[-1, 1]のfloat64列を生成する。"""

    values = np.empty(count, dtype=np.float64)
    state = seed & 0xFFFFFFFF
    for index in range(count):
        state = ((1664525 * state) + 1013904223) & 0xFFFFFFFF
        values[index] = ((state >> 8) / 16777215.0) * 2.0 - 1.0
    return values, state


def fixed_encoder(seed: int = ENCODER_SEED) -> tuple[np.ndarray, np.ndarray]:
    """固定seedから32→16エンコーダの重みとbiasを再生成する。

    重みは[-1, 1]、biasは[-0.5, 0.5]の一様乱数とする。デコーダだけが学習対象である。
    """

    values, state = _lcg_uniform(seed, HIDDEN_UNITS * FEATURE_BINS)
    biases, _ = _lcg_uniform(state, HIDDEN_UNITS)
    return values.reshape(HIDDEN_UNITS, FEATURE_BINS), biases * 0.5


def normalize_feature(frame: np.ndarray) -> np.ndarray:
    """int8 log-melの各binを±64でclipして[0, 1]へ写す。"""

    values = np.asarray(frame, dtype=np.int8)
    if values.shape != (FEATURE_BINS,):
        raise ValueError(f"expected ({FEATURE_BINS},), got {values.shape}")
    return (np.clip(values.astype(np.float64), -64.0, 64.0) + 64.0) / 128.0


def hard_sigmoid(values: np.ndarray) -> np.ndarray:
    """固定契約のHard Sigmoid: clip(0.2x + 0.5, 0, 1)。"""

    return np.clip((0.2 * np.asarray(values, dtype=np.float64)) + 0.5, 0.0, 1.0)


@dataclass
class BackgroundModel:
    """固定エンコーダとRLSデコーダからなる32→16→32背景モデル。"""

    decoder: np.ndarray
    inverse_correlation: np.ndarray
    encoder_weights: np.ndarray
    encoder_bias: np.ndarray

    @classmethod
    def create(cls, seed: int = ENCODER_SEED) -> "BackgroundModel":
        weights, bias = fixed_encoder(seed)
        return cls(
            decoder=np.zeros((FEATURE_BINS, HIDDEN_UNITS), dtype=np.float64),
            inverse_correlation=np.eye(HIDDEN_UNITS, dtype=np.float64) * RLS_INITIAL_DIAGONAL,
            encoder_weights=weights,
            encoder_bias=bias,
        )

    def hidden(self, frame: np.ndarray) -> np.ndarray:
        return hard_sigmoid(self.encoder_weights @ normalize_feature(frame) + self.encoder_bias)

    def reconstruction(self, frame: np.ndarray) -> np.ndarray:
        return self.decoder @ self.hidden(frame)

    def mse(self, frame: np.ndarray) -> float:
        residual = normalize_feature(frame) - self.reconstruction(frame)
        return float(np.mean(residual * residual))

    def update(self, frame: np.ndarray) -> float:
        """1フレーム分のRLS更新を行い、更新前の再構成MSEを返す。"""

        x = normalize_feature(frame)
        h = self.hidden(frame)
        reconstruction = self.decoder @ h
        residual = x - reconstruction
        p_h = self.inverse_correlation @ h
        gain = p_h / (RLS_FORGETTING_FACTOR + float(h @ p_h))
        self.decoder += np.outer(residual, gain)
        self.inverse_correlation = (
            self.inverse_correlation - np.outer(gain, h @ self.inverse_correlation)
        ) / RLS_FORGETTING_FACTOR
        return float(np.mean(residual * residual))

    def update_if_background(self, frame: np.ndarray) -> bool:
        """能動フレームを一つでも含む入力は背景モデルへ取り込まない。"""

        if is_active_frame(frame):
            return False
        self.update(frame)
        return True


def is_active_frame(frame: np.ndarray, threshold: float = ACTIVE_FRAME_STDDEV_LSB) -> bool:
    """フレーム内32binの母標準偏差が閾値を超えるときだけ能動とする。"""

    values = np.asarray(frame, dtype=np.int8)
    if values.shape != (FEATURE_BINS,):
        raise ValueError(f"expected ({FEATURE_BINS},), got {values.shape}")
    return bool(np.std(values.astype(np.float64), ddof=0) > threshold)


def _round_and_clip_int8(values: np.ndarray, low: int = -128) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    rounded = np.where(values >= 0.0, np.floor(values + 0.5), np.ceil(values - 0.5))
    return np.clip(rounded, low, 127).astype(np.int8)


def active_summary(patch: np.ndarray, minimum_frames: int = MIN_ACTIVE_FRAMES) -> np.ndarray | None:
    """能動フレームの{mean,std,max}を32binごとに連結した96次元int8を返す。

    `minimum_frames`未満は零ベクトルで代用せずNone（判定不能）を返す。mean/stdは最も近い
    int8へ半端をゼロから遠ざけて丸め、stdは0未満にならないようにする。
    """

    frames = np.asarray(patch, dtype=np.int8)
    if frames.ndim != 2 or frames.shape[1] != FEATURE_BINS:
        raise ValueError(f"expected (frames, {FEATURE_BINS}), got {frames.shape}")
    active = frames[np.asarray([is_active_frame(frame) for frame in frames], dtype=bool)]
    if active.shape[0] < minimum_frames:
        return None
    means = _round_and_clip_int8(np.mean(active, axis=0))
    stddevs = _round_and_clip_int8(np.std(active.astype(np.float64), axis=0, ddof=0), low=0)
    maxima = np.max(active, axis=0).astype(np.int8)
    return np.concatenate((means, stddevs, maxima))


def cosine_distance(left: np.ndarray, right: np.ndarray) -> float | None:
    """int8ベクトルのcosine距離（1-cosine）。零ベクトルは判定不能。"""

    a = np.asarray(left, dtype=np.int8).astype(np.float64)
    b = np.asarray(right, dtype=np.int8).astype(np.float64)
    if a.shape != (SUMMARY_DIMENSION,) or b.shape != (SUMMARY_DIMENSION,):
        raise ValueError(f"expected two ({SUMMARY_DIMENSION},) vectors")
    denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denominator == 0.0:
        return None
    return float(1.0 - np.clip((a @ b) / denominator, -1.0, 1.0))


def leave_one_out_threshold(samples: np.ndarray) -> float | None:
    """見本ごとの最近傍距離の平均+3σを、受理しきい値として返す。"""

    values = np.asarray(samples, dtype=np.int8)
    if values.ndim != 2 or values.shape[1] != SUMMARY_DIMENSION:
        raise ValueError(f"expected (samples, {SUMMARY_DIMENSION}), got {values.shape}")
    if values.shape[0] < 2:
        return None
    nearest: list[float] = []
    for index, sample in enumerate(values):
        distances = [
            distance
            for other_index, other in enumerate(values)
            if other_index != index
            if (distance := cosine_distance(sample, other)) is not None
        ]
        if not distances:
            return None
        nearest.append(min(distances))
    values_float = np.asarray(nearest, dtype=np.float64)
    return float(np.mean(values_float) + (3.0 * np.std(values_float, ddof=0)))


class Identification(Enum):
    INDETERMINATE = "indeterminate"
    NOT_READY = "not_ready"
    NOT_TARGET = "not_target"
    TARGET = "target"


@dataclass(frozen=True)
class IdentificationResult:
    status: Identification
    active_frame_count: int
    minimum_distance: float | None
    threshold: float | None


def identify(patch: np.ndarray, samples: np.ndarray, minimum_frames: int = MIN_ACTIVE_FRAMES) -> IdentificationResult:
    """能動フレーム不足と見本不足を、非対象と混同しない識別入口。"""

    frames = np.asarray(patch, dtype=np.int8)
    if frames.ndim != 2 or frames.shape[1] != FEATURE_BINS:
        raise ValueError(f"expected (frames, {FEATURE_BINS}), got {frames.shape}")
    active_count = sum(is_active_frame(frame) for frame in frames)
    summary = active_summary(frames, minimum_frames)
    if summary is None:
        return IdentificationResult(Identification.INDETERMINATE, active_count, None, None)
    threshold = leave_one_out_threshold(samples)
    if threshold is None:
        return IdentificationResult(Identification.NOT_READY, active_count, None, None)
    distances = [cosine_distance(summary, sample) for sample in np.asarray(samples, dtype=np.int8)]
    minimum = min(distance for distance in distances if distance is not None)
    status = Identification.TARGET if minimum <= threshold else Identification.NOT_TARGET
    return IdentificationResult(status, active_count, minimum, threshold)
