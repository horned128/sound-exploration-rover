import asyncio
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import socket
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from diagnostics import is_private_diagnostic_record

# ==========================================
# 設定: データ転送先 (UDP)
# ==========================================
# 受信したデータをそのまま転送する先のIPとポートのリスト
# 例: 別のPCで動く解析ソフトや、別のネットワーク上のロガーなど
FORWARD_DESTINATIONS = [
    # ("192.168.0.100", 5006),
    # ("127.0.0.1", 5007),
]

UDP_LISTEN_HOST = os.getenv("ROVER_MONITOR_UDP_HOST", "0.0.0.0")
UDP_LISTEN_PORT = int(os.getenv("ROVER_MONITOR_UDP_PORT", "5005"))
HTTP_LISTEN_HOST = os.getenv("ROVER_MONITOR_HTTP_HOST", "0.0.0.0")
HTTP_LISTEN_PORT = int(os.getenv("ROVER_MONITOR_HTTP_PORT", "8000"))
APPLICATION_DIRECTORY = Path(__file__).resolve().parent

# ==========================================
# ログ出力の設定 (JSON Lines .jsonl)
# ==========================================
LOG_DIRECTORY = Path(os.getenv("ROVER_MONITOR_LOG_DIR", APPLICATION_DIRECTORY / "logs")).resolve()
LOG_DIRECTORY.mkdir(parents=True, exist_ok=True)


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def resolve_log_path() -> Path:
    override = os.getenv("ROVER_MONITOR_LOG_PATH")
    if override:
        path = Path(override).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        return path
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return LOG_DIRECTORY / f"rover-monitor-{timestamp}.jsonl"


CURRENT_LOG_PATH = resolve_log_path()
log_file = CURRENT_LOG_PATH.open("a", encoding="utf-8", buffering=1)

# 最新ログファイルへの参照（シンボリックリンク作成、失敗時は無視）
latest_link = LOG_DIRECTORY / "rover-monitor-latest.jsonl"
try:
    if latest_link.is_symlink() or latest_link.exists():
        latest_link.unlink()
    latest_link.symlink_to(CURRENT_LOG_PATH.name)
except OSError:
    pass

logger = logging.getLogger("rover_monitor")
logger.setLevel(logging.INFO)
console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("[%(levelname)s] %(asctime)s - %(message)s"))
logger.addHandler(console_handler)


# ==========================================
# WebSocket マネージャー
# ==========================================
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []
        self.last_message: str | None = None

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        # クライアント接続時にログパス等のモニター情報を送信
        try:
            display_path = (
                str(CURRENT_LOG_PATH.relative_to(APPLICATION_DIRECTORY))
                if CURRENT_LOG_PATH.is_relative_to(APPLICATION_DIRECTORY)
                else str(CURRENT_LOG_PATH)
            )
        except ValueError:
            display_path = str(CURRENT_LOG_PATH)
        await websocket.send_text(
            json.dumps({
                "_type": "monitor_info",
                "log_path": display_path,
                "udp_port": UDP_LISTEN_PORT,
            })
        )
        if self.last_message is not None:
            await websocket.send_text(self.last_message)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        disconnected: list[WebSocket] = []
        self.last_message = message

        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                disconnected.append(connection)

        for connection in disconnected:
            self.disconnect(connection)


manager = ConnectionManager()

# ==========================================
# UDP 受信 & 転送プロトコル
# ==========================================
class UDPReceiverAndForwarder(asyncio.DatagramProtocol):
    def __init__(self):
        # 転送用のUDPソケットを作成
        self.forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def datagram_received(self, data: bytes, addr: tuple[str, int]):
        try:
            msg = data.decode("utf-8").strip()
            parsed = json.loads(msg)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            logger.warning("Invalid UDP telemetry from %s:%s: %s", addr[0], addr[1], error)
            return

        # 1. ログファイルへの書き込み (JSON Lines)
        if isinstance(parsed, dict):
            record = {"recorded_at": now_iso(), **parsed}
        else:
            record = {"recorded_at": now_iso(), "payload": parsed}

        log_file.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")

        # 走行中の特徴量・保存見本はJSONL専用で、通常状態WebSocketへ流さない。
        if is_private_diagnostic_record(parsed):
            return

        # 2. 指定された外部システムへデータをそのまま転送 (UDP Forwarding)
        for dest_ip, dest_port in FORWARD_DESTINATIONS:
            try:
                self.forward_sock.sendto(data, (dest_ip, dest_port))
            except Exception as e:
                print(f"Forwarding error to {dest_ip}:{dest_port} -> {e}")

        # 3. 接続しているWebブラウザへWebSocketで即時転送
        asyncio.create_task(manager.broadcast(msg))

    def connection_lost(self, exception):
        self.forward_sock.close()


# ==========================================
# FastAPI アプリケーション設定
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    # サーバー起動時にUDPのリスナーを開始
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPReceiverAndForwarder(),
        local_addr=(UDP_LISTEN_HOST, UDP_LISTEN_PORT),
    )
    print(f"UDP telemetry: {UDP_LISTEN_HOST}:{UDP_LISTEN_PORT}")
    print(f"Web monitor: http://127.0.0.1:{HTTP_LISTEN_PORT}")
    print(f"JSON Lines log: {CURRENT_LOG_PATH}")
    if FORWARD_DESTINATIONS:
        print(f"Data will be forwarded to: {FORWARD_DESTINATIONS}")

    yield
    # サーバー終了時にUDPポートとログファイルを閉じる
    transport.close()
    log_file.close()


app = FastAPI(lifespan=lifespan)


@app.get("/")
async def get():
    with (APPLICATION_DIRECTORY / "index.html").open("r", encoding="utf-8") as f:
        return HTMLResponse(f.read())


@app.get("/healthz")
async def healthz():
    return {
        "status": "ok",
        "udp_port": UDP_LISTEN_PORT,
        "log_path": str(CURRENT_LOG_PATH),
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)


if __name__ == "__main__":
    uvicorn.run(app, host=HTTP_LISTEN_HOST, port=HTTP_LISTEN_PORT)
