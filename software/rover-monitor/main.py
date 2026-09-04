import asyncio
import json
import logging
import os
import socket
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

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
# ログ出力の設定 (telemetry.log に追記)
# ==========================================
logger = logging.getLogger("telemetry")
logger.setLevel(logging.INFO)
file_handler = logging.FileHandler(APPLICATION_DIRECTORY / "rover-monitor.log", encoding="utf-8")
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logger.addHandler(file_handler)

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

    def datagram_received(self, data, addr):
        try:
            msg = data.decode("utf-8").strip()
            json.loads(msg)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            logger.warning("Invalid UDP telemetry from %s:%s: %s", addr[0], addr[1], error)
            return
        
        # 1. ログファイルへの書き込み
        logger.info(msg)
        
        # 2. 指定された外部システムへデータをそのまま転送 (UDP Forwarding)
        for dest_ip, dest_port in FORWARD_DESTINATIONS:
            try:
                self.forward_sock.sendto(data, (dest_ip, dest_port))
            except Exception as e:
                # 転送エラー時は標準出力に出す（ログファイルには混ぜない）
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
    if FORWARD_DESTINATIONS:
        print(f"Data will be forwarded to: {FORWARD_DESTINATIONS}")
        
    yield
    # サーバー終了時にUDPポートを閉じる
    transport.close()

app = FastAPI(lifespan=lifespan)

@app.get("/")
async def get():
    with (APPLICATION_DIRECTORY / "index.html").open("r", encoding="utf-8") as f:
        return HTMLResponse(f.read())


@app.get("/healthz")
async def healthz():
    return {"status": "ok", "udp_port": UDP_LISTEN_PORT}

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
