"""Sound Exploration Roverの固定数値契約に従うlog-mel参照実装。"""

from __future__ import annotations

import numpy as np

SAMPLE_RATE_HZ = 16_000
WINDOW_SAMPLES = 400
HOP_SAMPLES = 160
FFT_SIZE = 512
SPECTRUM_BINS = (FFT_SIZE // 2) + 1
MEL_BIN_COUNT = 32
MEL_LOW_HZ = 125.0
MEL_HIGH_HZ = 8_000.0
ENERGY_FLOOR = 1.0e-12
QUANTIZATION_SCALE = 0.125


def _hz_to_htk_mel(frequency_hz: np.ndarray | float) -> np.ndarray:
    return 2595.0 * np.log10(1.0 + (np.asarray(frequency_hz, dtype=np.float64) / 700.0))


def _htk_mel_to_hz(mel: np.ndarray | float) -> np.ndarray:
    return 700.0 * (np.power(10.0, np.asarray(mel, dtype=np.float64) / 2595.0) - 1.0)


def periodic_hann(dtype: np.dtype = np.dtype(np.float64)) -> np.ndarray:
    """400点periodic Hann係数を返す。"""

    if dtype == np.dtype(np.float32):
        sample = np.arange(WINDOW_SAMPLES, dtype=np.float64)
        return (0.5 - (0.5 * np.cos((2.0 * np.pi * sample) / WINDOW_SAMPLES))).astype(np.float32)
    sample = np.arange(WINDOW_SAMPLES, dtype=np.float64)
    return 0.5 - (0.5 * np.cos((2.0 * np.pi * sample) / WINDOW_SAMPLES))


def mel_filterbank() -> np.ndarray:
    """FFT bin中心で評価した非面積正規化HTK melフィルタを返す。"""

    low_mel = float(_hz_to_htk_mel(MEL_LOW_HZ))
    high_mel = float(_hz_to_htk_mel(MEL_HIGH_HZ))
    edges_hz = _htk_mel_to_hz(np.linspace(low_mel, high_mel, MEL_BIN_COUNT + 2, dtype=np.float64))
    frequencies_hz = np.arange(SPECTRUM_BINS, dtype=np.float64) * SAMPLE_RATE_HZ / FFT_SIZE
    filters = np.zeros((MEL_BIN_COUNT, SPECTRUM_BINS), dtype=np.float64)

    for mel_bin in range(MEL_BIN_COUNT):
        left_hz, center_hz, right_hz = edges_hz[mel_bin : mel_bin + 3]
        rising = (frequencies_hz >= left_hz) & (frequencies_hz < center_hz)
        falling = (frequencies_hz >= center_hz) & (frequencies_hz <= right_hz)
        filters[mel_bin, rising] = (frequencies_hz[rising] - left_hz) / (center_hz - left_hz)
        filters[mel_bin, falling] = (right_hz - frequencies_hz[falling]) / (right_hz - center_hz)
    return filters


def round_half_away_from_zero(values: np.ndarray) -> np.ndarray:
    """実数配列を中間値でゼロから遠ざかる方向へ丸める。"""

    values = np.asarray(values, dtype=np.float64)
    return np.where(values >= 0.0, np.floor(values + 0.5), np.ceil(values - 0.5))


def _firmware_mel_edges() -> np.ndarray:
    low_mel = float(_hz_to_htk_mel(MEL_LOW_HZ))
    high_mel = float(_hz_to_htk_mel(MEL_HIGH_HZ))
    mel = np.linspace(low_mel, high_mel, MEL_BIN_COUNT + 2, dtype=np.float64)
    return _htk_mel_to_hz(mel).astype(np.float32)


def _firmware_fft(windowed: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    real = np.zeros(FFT_SIZE, dtype=np.float32)
    imag = np.zeros(FFT_SIZE, dtype=np.float32)
    real[:WINDOW_SAMPLES] = windowed
    phase = -2.0 * np.pi * np.arange(FFT_SIZE // 2, dtype=np.float64) / FFT_SIZE
    twiddle_real = np.cos(phase).astype(np.float32)
    twiddle_imag = np.sin(phase).astype(np.float32)

    for index in range(FFT_SIZE):
        value = index
        reversed_index = 0
        for _ in range(9):
            reversed_index = (reversed_index << 1) | (value & 1)
            value >>= 1
        if reversed_index > index:
            real[index], real[reversed_index] = real[reversed_index], real[index]
            imag[index], imag[reversed_index] = imag[reversed_index], imag[index]

    length = 2
    while length <= FFT_SIZE:
        half_length = length // 2
        twiddle_step = FFT_SIZE // length
        for base in range(0, FFT_SIZE, length):
            for offset in range(half_length):
                twiddle_index = offset * twiddle_step
                odd_real = real[base + offset + half_length]
                odd_imag = imag[base + offset + half_length]
                rotated_real = np.float32(
                    (twiddle_real[twiddle_index] * odd_real) - (twiddle_imag[twiddle_index] * odd_imag)
                )
                rotated_imag = np.float32(
                    (twiddle_real[twiddle_index] * odd_imag) + (twiddle_imag[twiddle_index] * odd_real)
                )
                even_real = real[base + offset]
                even_imag = imag[base + offset]
                real[base + offset] = np.float32(even_real + rotated_real)
                imag[base + offset] = np.float32(even_imag + rotated_imag)
                real[base + offset + half_length] = np.float32(even_real - rotated_real)
                imag[base + offset + half_length] = np.float32(even_imag - rotated_imag)
        length <<= 1
    return real, imag


def extract_log_mel_frame(samples: np.ndarray) -> np.ndarray:
    """ESP32S3と同じfloat32演算順でint8特徴量を抽出する。"""

    pcm = np.asarray(samples)
    if pcm.shape != (WINDOW_SAMPLES,):
        raise ValueError(f"expected ({WINDOW_SAMPLES},), got {pcm.shape}")
    normalized = pcm.astype(np.float32) / np.float32(np.iinfo(np.int32).max)
    windowed = normalized * periodic_hann(np.dtype(np.float32))
    real, imag = _firmware_fft(windowed)
    power = (real[:SPECTRUM_BINS] * real[:SPECTRUM_BINS]) + (
        imag[:SPECTRUM_BINS] * imag[:SPECTRUM_BINS]
    )
    edges_hz = _firmware_mel_edges()
    log_energy = np.empty(MEL_BIN_COUNT, dtype=np.float32)
    mean = np.float32(0.0)
    for mel_bin in range(MEL_BIN_COUNT):
        left_hz, center_hz, right_hz = edges_hz[mel_bin : mel_bin + 3]
        energy = np.float32(0.0)
        for spectrum_bin in range(SPECTRUM_BINS):
            frequency_hz = np.float32(spectrum_bin * SAMPLE_RATE_HZ) / np.float32(FFT_SIZE)
            weight = np.float32(0.0)
            if left_hz <= frequency_hz < center_hz:
                weight = (frequency_hz - left_hz) / (center_hz - left_hz)
            elif center_hz <= frequency_hz <= right_hz:
                weight = (right_hz - frequency_hz) / (right_hz - center_hz)
            energy = np.float32(energy + np.float32(weight * power[spectrum_bin]))
        energy = max(energy, np.float32(ENERGY_FLOOR))
        log_energy[mel_bin] = np.log(energy, dtype=np.float32)
        mean = np.float32(mean + log_energy[mel_bin])
    mean = np.float32(mean / np.float32(MEL_BIN_COUNT))
    scaled = (log_energy - mean) / np.float32(QUANTIZATION_SCALE)
    quantized = round_half_away_from_zero(scaled)
    return np.clip(quantized, -128, 127).astype(np.int8)


def _extract_log_mel_frame_numpy(samples: np.ndarray) -> np.ndarray:
    """倍精度NumPy FFTによる数式確認用log-mel抽出。"""

    pcm = np.asarray(samples)
    if pcm.shape != (WINDOW_SAMPLES,):
        raise ValueError(f"expected ({WINDOW_SAMPLES},), got {pcm.shape}")
    if not np.issubdtype(pcm.dtype, np.integer):
        raise TypeError("samples must be an integer PCM array")

    normalized = pcm.astype(np.float64) / np.iinfo(np.int32).max
    windowed = normalized * periodic_hann()
    spectrum = np.fft.rfft(windowed, n=FFT_SIZE)
    power = (spectrum.real * spectrum.real) + (spectrum.imag * spectrum.imag)
    energy = mel_filterbank() @ power
    log_energy = np.log(np.maximum(energy, ENERGY_FLOOR))
    normalized_log_energy = log_energy - np.mean(log_energy)
    quantized = round_half_away_from_zero(normalized_log_energy / QUANTIZATION_SCALE)
    return np.clip(quantized, -128, 127).astype(np.int8)


def extract_log_mel_stream(samples: np.ndarray) -> np.ndarray:
    """連続PCMを400点窓・160点hopで特徴量列へ変換する。"""

    pcm = np.asarray(samples)
    if pcm.ndim != 1:
        raise ValueError("samples must be one-dimensional")
    if pcm.size < WINDOW_SAMPLES:
        return np.empty((0, MEL_BIN_COUNT), dtype=np.int8)
    frames = [
        extract_log_mel_frame(pcm[start : start + WINDOW_SAMPLES])
        for start in range(0, pcm.size - WINDOW_SAMPLES + 1, HOP_SAMPLES)
    ]
    return np.stack(frames)
