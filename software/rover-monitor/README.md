# Rover Monitor

ESP32S3からのUDP診断テレメトリを受信し、ブラウザで確認するローカル監視ツールです。

## 起動

```powershell
cd software/rover-monitor
uv run main.py
```

次の表示を確認して、ブラウザで <http://127.0.0.1:8000> を開きます。

```text
UDP telemetry: 0.0.0.0:5005
Web monitor: http://127.0.0.1:8000
```

UDPポートは5005です。ESP32S3のWi-Fiテレメトリ送信先を、このPCのIPアドレスと5005へ設定します。

別のポートを使う場合は、環境変数で変更できます。

```powershell
$env:ROVER_MONITOR_UDP_PORT = "5006"
$env:ROVER_MONITOR_HTTP_PORT = "8080"
uv run main.py
```

## 表示内容

- 音響・Wi-Fi・USB・CPU0/CPU1指令状態
- 左右モーターRPMおよびPWM Duty比（All / RPM / Duty モード切替対応）、操舵角と音源方向の円形方位図、音量の時系列
- ToF LEFT / CENTER / RIGHTの距離、センサー有効フラグ、エラー、現在の走行ルール
- BMI270の加速度と角速度
- 車体を原点とする局所障害物マップ

局所障害物マップは、前方を上としてLEFT / CENTER / RIGHTのToF測距値を描画します。自己位置や向きの推定は含まれないため、これはSLAMの地図ではなく、現在の車体周囲を確認するための表示です。
