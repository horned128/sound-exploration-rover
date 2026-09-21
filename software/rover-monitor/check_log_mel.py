#!/usr/bin/env python3
"""Rover MonitorログからESP32S3 log-mel実機診断を判定する。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

MIN_CAPTURE_MS = 5_000
MIN_FEATURE_FPS = 90.0
MAX_FEATURE_FPS = 110.0
MAX_BLOCK_US = 16_000


def load_samples(path: Path) -> list[tuple[int, dict[str, object]]]:
    samples: list[tuple[int, dict[str, object]]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        json_start = line.find("{")
        if json_start < 0:
            continue
        try:
            message = json.loads(line[json_start:])
        except json.JSONDecodeError:
            continue
        diagnostics = message.get("esp_audio")
        if isinstance(diagnostics, dict) and isinstance(message.get("esp_ms"), int):
            esp_ms = message["esp_ms"]
            feature_frames = diagnostics.get("feature_frames")
            if samples:
                previous_ms, previous_diagnostics = samples[-1]
                previous_frames = previous_diagnostics.get("feature_frames")
                if (esp_ms < previous_ms) or (
                    isinstance(feature_frames, int)
                    and isinstance(previous_frames, int)
                    and feature_frames < previous_frames
                ):
                    samples.clear()
            samples.append((esp_ms, diagnostics))
    if not samples:
        raise ValueError(f"{path}: esp_audioを含むJSONログがありません")
    return samples


def assess(samples: list[tuple[int, dict[str, object]]]) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    details: list[str] = []
    first_ms, first = samples[0]
    last_ms, last = samples[-1]
    elapsed_ms = last_ms - first_ms
    first_frames = int(first.get("feature_frames", 0))
    last_frames = int(last.get("feature_frames", 0))
    generated_frames = last_frames - first_frames
    average_fps = (generated_frames * 1000.0 / elapsed_ms) if elapsed_ms > 0 else 0.0
    first_overruns = int(first.get("i2s_overruns", 0))
    last_overruns = int(last.get("i2s_overruns", 0))
    maximum_block_us = max(int(sample.get("log_mel_block_max_us", 0)) for _, sample in samples)
    self_test_pass = all(sample.get("self_test_pass") is True for _, sample in samples)
    ring_frames = int(last.get("ring_frames", 0))

    details.append(f"samples={len(samples)} duration={elapsed_ms / 1000.0:.2f}s")
    details.append(f"feature_rate={average_fps:.2f}fps frames={first_frames}->{last_frames}")
    details.append(f"ring_frames={ring_frames}/80 block_max={maximum_block_us}us")
    details.append(f"i2s_overruns={first_overruns}->{last_overruns} self_test={'PASS' if self_test_pass else 'FAIL'}")

    if elapsed_ms < MIN_CAPTURE_MS:
        failures.append(f"計測時間が{MIN_CAPTURE_MS / 1000:.0f}秒未満です")
    if not self_test_pass:
        failures.append("固定ベクトル自己テストが失敗しています")
    if not (MIN_FEATURE_FPS <= average_fps <= MAX_FEATURE_FPS):
        failures.append(f"平均特徴量レート{average_fps:.2f}fpsが{MIN_FEATURE_FPS:.0f}..{MAX_FEATURE_FPS:.0f}fps外です")
    if ring_frames != 80:
        failures.append(f"特徴量リングが未充填です: {ring_frames}/80")
    if maximum_block_us >= MAX_BLOCK_US:
        failures.append(f"最大log-mel処理時間{maximum_block_us}usが{MAX_BLOCK_US}us以上です")
    if last_overruns != first_overruns:
        failures.append(f"I2S overrunが計測中に増加しました: {first_overruns}->{last_overruns}")
    return failures, details


def find_default_log() -> Path:
    base_dir = Path(__file__).resolve().parent
    logs_dir = base_dir / "logs"
    if logs_dir.is_dir():
        jsonl_files = sorted(logs_dir.glob("rover-monitor-*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
        if jsonl_files:
            return jsonl_files[0]
        all_jsonls = sorted(logs_dir.glob("*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
        if all_jsonls:
            return all_jsonls[0]
    return base_dir / "rover-monitor.log"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, default=None, help="ログファイルパス (.jsonl または .log)")
    arguments = parser.parse_args()
    log_path = arguments.log or find_default_log()
    print(f"Reading log: {log_path}")
    try:
        failures, details = assess(load_samples(log_path))
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1

    for detail in details:
        print(detail)
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("PASS: ESP32S3 log-mel実機診断")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
