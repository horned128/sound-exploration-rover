"""Local dashboard and durable JSONL recorder for the Acoustic AI Lab link."""

from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import struct
import threading
import time
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import serial
import uvicorn

from protocol import (
    COMMAND_LEARNING_CANCEL,
    COMMAND_LEARNING_COMMIT,
    COMMAND_LEARNING_START,
    COMMAND_PROFILE_READ,
    COMMAND_STATUS,
    Frame,
    FrameParser,
    MESSAGE_COMMAND_RESULT,
    MESSAGE_PROFILE_CHUNK,
    MESSAGE_SNAPSHOT,
    MESSAGE_SUMMARY_CHUNK,
    decode_command_result,
    decode_profile_chunk,
    decode_snapshot,
    decode_summary_chunk,
    make_command,
)

ROOT = Path(__file__).resolve().parent
LOG_DIRECTORY = ROOT / "logs"
COMMANDS = {
    "status": COMMAND_STATUS,
    "learn_start": COMMAND_LEARNING_START,
    "learn_commit": COMMAND_LEARNING_COMMIT,
    "learn_cancel": COMMAND_LEARNING_CANCEL,
    "profile_read": COMMAND_PROFILE_READ,
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class SerialWorker:
    """Owns blocking pyserial I/O; decoded frames are returned to the event loop."""

    def __init__(self, port: str, hub: "LabHub") -> None:
        self.port = port
        self.hub = hub
        self._outbox: queue.Queue[bytes] = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="acoustic-ai-lab-usb", daemon=True)
        self.connected = False

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.5)

    def send(self, data: bytes) -> None:
        self._outbox.put(data)

    def _publish(self, kind: str, body: dict[str, Any]) -> None:
        self.hub.loop.call_soon_threadsafe(self.hub.schedule_record, kind, body)

    def _run(self) -> None:
        parser = FrameParser()
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, 115200, timeout=0.15, write_timeout=0.5) as device:
                    self.connected = True
                    self._publish("link", {"connected": True, "port": self.port})
                    while not self._stop.is_set():
                        try:
                            while True:
                                device.write(self._outbox.get_nowait())
                        except queue.Empty:
                            pass
                        incoming = device.read(device.in_waiting or 1)
                        for frame in parser.feed(incoming):
                            self.hub.loop.call_soon_threadsafe(self.hub.schedule_frame, frame)
            except serial.SerialException as error:
                self.connected = False
                self._publish("link", {"connected": False, "port": self.port, "error": str(error)})
                self._stop.wait(1.0)
            finally:
                if self.connected:
                    self.connected = False
                    self._publish("link", {"connected": False, "port": self.port})


class LabHub:
    def __init__(self, loop: asyncio.AbstractEventLoop, port: str | None) -> None:
        LOG_DIRECTORY.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.log_path = LOG_DIRECTORY / f"acoustic-ai-lab-{timestamp}.jsonl"
        self._log = self.log_path.open("a", encoding="utf-8", buffering=1)
        self.loop = loop
        self.port = port
        self.worker: SerialWorker | None = SerialWorker(port, self) if port else None
        self.connections: set[WebSocket] = set()
        self.snapshot: dict[str, Any] | None = None
        self.current_summary: list[int] | None = None
        self.current_summary_generation: int | None = None
        self._summary_chunks: dict[int, dict[int, list[int]]] = {}
        self.profile: dict[int, list[int]] = {}
        self._profile_chunks: dict[tuple[int, int], dict[int, list[int]]] = {}
        self.command_sequence = 1
        self.last_event_at: str | None = None
        self.link: dict[str, Any] = {"connected": False, "port": port}

    def close(self) -> None:
        if self.worker:
            self.worker.stop()
        self._log.close()

    def schedule_record(self, kind: str, body: dict[str, Any]) -> None:
        asyncio.create_task(self.record(kind, body))

    def schedule_frame(self, frame: Frame) -> None:
        asyncio.create_task(self.handle_frame(frame))

    async def record(self, kind: str, body: dict[str, Any], frame: Frame | None = None) -> None:
        record: dict[str, Any] = {"recorded_at": now_iso(), "kind": kind, **body}
        if frame:
            record.update({"sequence": frame.sequence, "uptime_ms": frame.uptime_ms, "message_type": frame.message_type})
        self.last_event_at = record["recorded_at"]
        self._log.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        if kind == "link":
            self.link = body
        await self.broadcast({"kind": kind, "record": record, "state": self.state()})

    def state(self) -> dict[str, Any]:
        return {
            "port": self.port,
            "link": {**self.link, "connected": bool(self.worker and self.worker.connected)},
            "log_path": str(self.log_path),
            "last_event_at": self.last_event_at,
            "snapshot": self.snapshot,
            "current_summary": self.current_summary,
            "current_summary_generation": self.current_summary_generation,
            "profile": self.profile,
        }

    async def handle_frame(self, frame: Frame) -> None:
        try:
            if frame.message_type == MESSAGE_SNAPSHOT:
                self.snapshot = decode_snapshot(frame.payload)
                await self.record("snapshot", self.snapshot, frame)
            elif frame.message_type == MESSAGE_SUMMARY_CHUNK:
                decoded = decode_summary_chunk(frame.payload)
                chunks = self._summary_chunks.setdefault(decoded["feature_generation"], {})
                chunks[decoded["chunk_index"]] = decoded["data"]
                if len(chunks) == decoded["chunk_count"]:
                    self.current_summary_generation = decoded["feature_generation"]
                    self.current_summary = [value for index in range(decoded["chunk_count"]) for value in chunks[index]]
                    self._summary_chunks = {decoded["feature_generation"]: chunks}
                await self.record("summary_chunk", decoded, frame)
            elif frame.message_type == MESSAGE_PROFILE_CHUNK:
                decoded = decode_profile_chunk(frame.payload)
                key = (decoded["storage_generation"], decoded["sample_index"])
                chunks = self._profile_chunks.setdefault(key, {})
                chunks[decoded["chunk_index"]] = decoded["data"]
                if len(chunks) == decoded["chunk_count"]:
                    self.profile[decoded["sample_index"]] = [
                        value for index in range(decoded["chunk_count"]) for value in chunks[index]
                    ]
                await self.record("profile_chunk", decoded, frame)
            elif frame.message_type == MESSAGE_COMMAND_RESULT:
                await self.record("command_result", decode_command_result(frame.payload), frame)
            else:
                await self.record("unhandled_frame", {"payload_hex": frame.payload.hex()}, frame)
        except (KeyError, ValueError, struct.error) as error:  # type: ignore[name-defined]
            await self.record("decode_error", {"error": str(error), "payload_hex": frame.payload.hex()}, frame)

    async def broadcast(self, message: dict[str, Any]) -> None:
        serialized = json.dumps(message, ensure_ascii=False)
        stale: list[WebSocket] = []
        for websocket in self.connections:
            try:
                await websocket.send_text(serialized)
            except RuntimeError:
                stale.append(websocket)
        for websocket in stale:
            self.connections.discard(websocket)

    async def command(self, name: str) -> None:
        if name not in COMMANDS:
            raise ValueError("unsupported command")
        if not self.worker:
            raise RuntimeError("no USB serial port was configured")
        sequence = self.command_sequence
        self.command_sequence += 1
        self.worker.send(make_command(COMMANDS[name], sequence))
        await self.record("command_sent", {"command_name": name, "command": COMMANDS[name], "sequence": sequence})


def create_app(port: str | None) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        hub = LabHub(asyncio.get_running_loop(), port)
        app.state.hub = hub
        if hub.worker:
            hub.worker.start()
        try:
            yield
        finally:
            hub.close()

    app = FastAPI(title="Acoustic AI Lab", lifespan=lifespan)

    @app.get("/")
    async def root() -> FileResponse:
        return FileResponse(ROOT / "index.html")

    @app.get("/api/state")
    async def state() -> dict[str, Any]:
        return app.state.hub.state()

    @app.post("/api/command/{name}")
    async def command(name: str) -> dict[str, str]:
        try:
            await app.state.hub.command(name)
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        return {"status": "queued", "command": name}

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        hub: LabHub = app.state.hub
        hub.connections.add(websocket)
        await websocket.send_json({"kind": "state", "state": hub.state()})
        try:
            while True:
                request = await websocket.receive_json()
                if request.get("command"):
                    try:
                        await hub.command(str(request["command"]))
                    except (ValueError, RuntimeError) as error:
                        await websocket.send_json({"kind": "command_error", "error": str(error)})
        except WebSocketDisconnect:
            pass
        finally:
            hub.connections.discard(websocket)

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Acoustic AI Lab USB dashboard")
    parser.add_argument("--port", default=os.getenv("ACOUSTIC_AI_LAB_PORT"), help="J11 CDC port, e.g. /dev/cu.usbmodem*.")
    parser.add_argument("--http-port", type=int, default=int(os.getenv("ACOUSTIC_AI_LAB_HTTP_PORT", "8010")))
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    uvicorn.run(create_app(arguments.port), host="127.0.0.1", port=arguments.http_port, log_level="info")
