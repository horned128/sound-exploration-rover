import asyncio
import json
import logging
import socket
from contextlib import asynccontextmanager
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

# ==========================================
# ログ出力の設定 (telemetry.log に追記)
# ==========================================
logger = logging.getLogger("telemetry")
logger.setLevel(logging.INFO)
file_handler = logging.FileHandler("rover-monitor.log", encoding="utf-8")
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logger.addHandler(file_handler)

# ==========================================
# WebSocket マネージャー
# ==========================================
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                pass

manager = ConnectionManager()

# ==========================================
# UDP 受信 & 転送プロトコル
# ==========================================
class UDPReceiverAndForwarder(asyncio.DatagramProtocol):
    def __init__(self):
        # 転送用のUDPソケットを作成
        self.forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def datagram_received(self, data, addr):
        msg = data.decode('utf-8').strip()
        
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

# ==========================================
# FastAPI アプリケーション設定
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    # サーバー起動時にUDPのリスナーを開始
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPReceiverAndForwarder(),
        local_addr=("0.0.0.0", 5005)
    )
    print("UDP Server listening on 0.0.0.0:5005...")
    if FORWARD_DESTINATIONS:
        print(f"Data will be forwarded to: {FORWARD_DESTINATIONS}")
        
    yield
    # サーバー終了時にUDPポートを閉じる
    transport.close()

app = FastAPI(lifespan=lifespan)

@app.get("/")
async def get():
    with open("index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(f.read())

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)