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
- 学習モード、MRAM保存データ有無、直近の保存結果
- 車体を原点とする局所障害物マップ
- XVF3800のraw / 循環平均DoA、DoA信頼度、複数方位線とオドメトリからの音源位置推定
- 音源位置の残差・交差角・観測基線、到着候補／確認状態、最小並進回頭の診断
- ESP32S3 log-melの起動時自己テスト、特徴量生成fps、80フレームリング、
  256-sampleブロックの直近／最大処理時間、I2S overrun

局所障害物マップは、前方を上としてLEFT / CENTER / RIGHTのToF測距値を描画します。自己位置や向きの推定は含まれないため、これはSLAMの地図ではなく、現在の車体周囲を確認するための表示です。

ローバー中心ビューのオレンジ線（直近DoA）と累積相対マップの音響イベント線は、CPU0と同じ車体正面0°・右正の向きへ`audio.doa_deg`を変換して描画します。2026-09-24の四方試験では、右前のDoA 303°→右+57°、左前58°→左-58°、右後141°→右+141°、左後237°→左-123°でした。青線の`navigation.bearing_deg`はCPU0の右正の値をそのまま描画し、再反転しません。数値欄と保存ログは受信した値のままです。真横付近（制御用DoA 90°/270°）は未較正です。

音源位置は、CPU0が複数のDoA観測、BMI270の方位、車輪オドメトリを用いて推定します。単一DoAから距離を生成せず、十分な観測基線・交差角・残差が揃わない間は走行目標を無効として表示します。推定位置は累積相対マップ上に水色の`SOURCE`点として描画されます。

ESP32S3からのUDP JSONには、既存の`ROVER_TELEMETRY`に加えてCPU0の`NAV_DIAGNOSTICS`フレーム由来の`navigation`、`arrival`、`escape`オブジェクトが含まれます。`main.py`はJSONを検証・転送するだけなので、追加フィールドのための個別変更は不要です。

## ログ保存とlog-mel実機確認

ESP32S3を書き込んでRover Monitorを30秒以上動かすと、テレメトリが `logs/rover-monitor-YYYYMMDD-HHMMSS.jsonl` （JSON Lines形式）へ自動保存されます。
画面ヘッダーにも記録中のファイル名が表示されます。
CPU0が未接続で`cpu_valid:false`の場合も、`esp_audio`診断は記録されます。

走行中のAI診断は通常状態JSONと別datagramで受信し、`record_type`が`acoustic_diagnostic`の行に特徴量generationごとの192次元要約と推論結果・距離を、`acoustic_sample`の行に起動時／再学習後の保存見本を記録します。baseline制御では`match_state=UNKNOWN`、`reason=baseline_policy`です。これらの行はJSONL専用で、通常状態WebSocketには転送しません。

| 項目 | 合格条件 |
|---|---|
| `esp_audio.self_test_pass` | 常に`true` |
| `esp_audio.feature_fps_x100` | 起動直後を除き概ね`9000..11000` |
| `esp_audio.ring_frames` | 約800 ms後に`80` |
| `esp_audio.log_mel_block_max_us` | I2Sの16 ms読出し周期より短い`16000`未満 |
| `esp_audio.i2s_overruns` | 計測中に増加しない |

画面の`Log-mel DSP`欄で同じ値を確認できます。詳細解析用には `logs/` 配下の `.jsonl` ファイルをそのまま渡してください。
ローカルで自動判定する場合は、Rover Monitorを停止してから次を実行します（引数省略時は最新の `.jsonl` を自動検出します）。

```powershell
uv run python check_log_mel.py
```

特定のログファイルを指定して判定することも可能です。

```powershell
uv run python check_log_mel.py logs/rover-monitor-20260921-220000.jsonl
```

## MRAM学習結果の実機確認

CPU0とESP32S3を書き込んだ後、EK-RA8P1のSW1を2秒間長押しすると学習を開始する。対象音を1〜5回取り込み、再度SW1を2秒間長押しすると平均プロトタイプをCode MRAMへ保存する。`logs/` の `.jsonl` に記録された `learning` を確認する。

| 項目 | 合格条件 |
|---|---|
| `learning.active` | 学習中は`1`、保存後は`0` |
| `learning.storage_valid` | 保存後と再起動後に`1` |
| `learning.storage_result` | 保存後と再起動後に`0` |

`storage_result`は、`0`: 成功、`1`: 有効データなし、`2`: 未初期化、`3`: 引数、`4`: 領域、`5`: ドライバ開始、`6`: CPU1停止、`7`: blank処理、`8`: 書込み、`9`: 読戻し検証の各結果を表す。初回起動直後の`1`は正常で、保存後または保存済み状態での再起動後に`0`となる。
