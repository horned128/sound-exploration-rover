"""Loss-aware conversion of rover-monitor acoustic_pcm JSONL into mono 16-kHz WAV.

Each contiguous run becomes one WAV; packet gaps are NEVER filled with silence.
Optional annotations JSON contains {"log-name.jsonl": [{"start_s": 1, "end_s": 3,
"label": "target", "session": "run1"}, ...]}. Boundaries are sample-clock seconds,
relative to ESP boot (not wall time). Annotated clips crossing packet loss are rejected.
"""
from __future__ import annotations

import argparse
import base64
import json
import statistics
import wave
from datetime import datetime
from pathlib import Path

RATE = 16000


def _assemble(chunks: dict[int, bytes], repeated: int, path: Path) -> tuple[list[tuple[int, bytes]], dict]:
    runs: list[tuple[int, bytes]] = []
    start = previous_end = None
    samples = bytearray()
    for first, raw in sorted(chunks.items()):
        if previous_end is not None and first < previous_end:
            raise ValueError(f"overlapping PCM blocks: {path}:{first}")
        if first != previous_end:
            if samples:
                runs.append((start, bytes(samples)))
            start = first
            samples = bytearray()
        samples.extend(raw)
        previous_end = first + len(raw) // 2
    if samples:
        runs.append((start, bytes(samples)))
    return runs, {"blocks": len(chunks), "duplicate_blocks": repeated,
                   "runs": len(runs), "missing_samples_between_runs": sum(
                       b[0] - (a[0] + len(a[1]) // 2) for a, b in zip(runs, runs[1:]))}


def pcm_sessions(path: Path) -> list[dict]:
    """Separate ESP reboots, whose first_sample counter restarts in one JSONL."""
    sessions: list[dict] = []
    chunks: dict[int, bytes] = {}
    repeated = 0
    audio_clock_anchors: list[float] = []
    last_ms = last_sample = None

    def finish() -> None:
        nonlocal chunks, repeated, audio_clock_anchors
        if chunks:
            runs, stats = _assemble(chunks, repeated, path)
            entry = {"index": len(sessions), "runs": runs, "stats": stats}
            if audio_clock_anchors:
                entry["audio_clock_anchor_epoch_s"] = statistics.median(audio_clock_anchors)
            sessions.append(entry)
        chunks, repeated, audio_clock_anchors = {}, 0, []

    with path.open(encoding="utf-8") as source:
        for line in source:
            record = json.loads(line)
            if record.get("record_type") != "acoustic_pcm":
                continue
            if record.get("schema") != 1 or not 0 < record.get("sample_count", 0) <= 256:
                raise ValueError(f"invalid PCM metadata: {path}")
            raw = base64.b64decode(record["pcm16le_b64"], validate=True)
            if len(raw) != record["sample_count"] * 2:
                raise ValueError(f"invalid PCM length: {path}")
            first = record["first_sample"]
            ms = record.get("esp_ms")
            if chunks and ((ms is not None and last_ms is not None and ms + 1000 < last_ms) or
                           (last_sample is not None and first + 16000 < last_sample)):
                finish()
            if first in chunks:
                repeated += 1
                if chunks[first] != raw:
                    raise ValueError(f"conflicting duplicate PCM within ESP session: {path}:{first}")
            chunks[first] = raw
            if "recorded_at" in record:
                audio_clock_anchors.append(datetime.fromisoformat(record["recorded_at"]).timestamp() - first / RATE)
            last_ms, last_sample = ms, first
    finish()
    return sessions


def pcm_runs(path: Path) -> tuple[list[tuple[int, bytes]], dict]:
    """Compatibility helper for a single ESP uptime; never join rebooted audio."""
    sessions = pcm_sessions(path)
    if len(sessions) > 1:
        raise ValueError(f"multiple ESP sessions in {path}: use pcm_sessions()")
    if not sessions:
        return [], {"blocks": 0, "duplicate_blocks": 0, "runs": 0, "missing_samples_between_runs": 0}
    return sessions[0]["runs"], sessions[0]["stats"]


def save_wav(path: Path, data: bytes) -> None:
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(RATE)
        output.writeframes(data)


def convert(paths: list[Path], output: Path, annotations: dict | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    manifest: list[dict] = []
    stats: dict = {}
    for path in paths:
        sessions = pcm_sessions(path)
        stats[path.name] = (sessions[0]["stats"] if len(sessions) == 1 else
                            {"sessions": [{"index": s["index"], **s["stats"]} for s in sessions],
                             "blocks": sum(s["stats"]["blocks"] for s in sessions),
                             "runs": sum(s["stats"]["runs"] for s in sessions),
                             "missing_samples_between_runs": sum(
                                 s["stats"]["missing_samples_between_runs"] for s in sessions)})
        clips = (annotations or {}).get(path.name, [])
        if clips:
            for index, clip in enumerate(clips):
                start = round(clip["start_s"] * RATE)
                end = round(clip["end_s"] * RATE)
                if end <= start or clip["label"] not in {"target", "target_possible", "background", "motor", "tv", "speech", "other"}:
                    raise ValueError(f"bad annotation: {clip}")
                if len(sessions) > 1 and "session_index" not in clip:
                    raise ValueError(f"annotation needs session_index after ESP restart: {path.name}")
                session_index = clip.get("session_index", 0)
                if not 0 <= session_index < len(sessions):
                    raise ValueError(f"invalid PCM session_index: {path.name}:{session_index}")
                containing = [(first, raw) for first, raw in sessions[session_index]["runs"] if first <= start and
                              first + len(raw) // 2 >= end]
                if len(containing) != 1:
                    raise ValueError(f"annotation overlaps missing audio: {path.name}:{start}-{end}")
                first, raw = containing[0]
                data = raw[(start - first) * 2:(end - first) * 2]
                name = f"{path.stem}-session-{session_index}-clip-{index:03d}.wav"
                save_wav(output / name, data)
                manifest.append({"wav": name, "log": path.name, "start_sample": start,
                                 "end_sample": end, "label": clip["label"],
                                 "session_index": session_index,
                                 "session": clip.get("session", path.stem)})
        else:
            for s in sessions:
                for index, (first, raw) in enumerate(s["runs"]):
                    if len(raw) < RATE:  # very short discontinuities remain in stats
                        continue
                    name = f"{path.stem}-session-{s['index']}-run-{index:03d}.wav"
                    save_wav(output / name, raw)
                    manifest.append({"wav": name, "log": path.name, "start_sample": first,
                                     "end_sample": first + len(raw) // 2, "label": None,
                                     "session_index": s["index"], "session": path.stem})
    report = {"sample_rate": RATE, "files": stats, "clips": manifest,
              "note": "unlabeled clips cannot be used as positives or negatives"}
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--annotations", type=Path)
    args = parser.parse_args()
    annotations = json.loads(args.annotations.read_text(encoding="utf-8")) if args.annotations else None
    report = convert(args.logs, args.output, annotations)
    print(json.dumps({"clips": len(report["clips"]), "files": report["files"]}))


if __name__ == "__main__":
    main()
