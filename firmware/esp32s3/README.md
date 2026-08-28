# ReSpeaker XIAO ESP32S3 音響フロントエンド

`reSpeaker XVF3800 USB 4-Mic Array with XIAO ESP32S3` のXIAO側で動作するESP-IDFプロジェクトです。XVF3800が推定したDoA/VADと、I2S音声から計算した相対レベルを、単一のUSB CDC-ACMインターフェースでEK-RA8P1へ送信します。CPU0から返る診断snapshotはWi-Fi UDPのJSON LinesとしてPCへ転送します。

音響取得、USB通信、Wi-Fi診断を別taskに分離し、Wi-Fi再接続やUDP送信でUSBの音響周期を待たせない構成です。

## 実装範囲

- XVF3800 I2C: SDA GPIO5、SCL GPIO6、7-bit address `0x2C`、100 kHz
- XVF3800 I2S: WS GPIO7、BCLK GPIO8、DIN GPIO43、DOUT GPIO44
- I2S形式: XIAO master、16 kHz、stereo、32 bit、Philips I2S
- USB: XIAO側USB-CをTinyUSBのCDC-ACM deviceとして使用
- 送信: `HELLO`（接続時と1 s周期）、`ACOUSTIC_OBSERVATION`（50 ms周期）、`HEALTH`（1 s周期）
- 受信: CPU0の`ROVER_TELEMETRY`（250 ms周期）
- Wi-Fi: station mode、自動再接続、指定PCへのUDP JSON Lines（250 ms周期）
- フレーム: `firmware/common/acoustic_protocol.h` の共有バイナリプロトコル

レベル値はI2S PCMから計算した `dBFS × 100` です。音圧レベル（dB SPL）ではなく、XVF3800のAGCや音響処理後の相対値なので、実機環境で走行開始閾値を調整してください。

観測のI2S overrun/I2C error flagは現在の観測区間に対する一時値で、正常化すれば次の観測で解除します。I2Sの最新captureが100 ms以上更新されなければI2S staleを立て、level/peakを最小値へ落とします。`HEALTH.i2s_overrun_count`と`i2c_error_count`は起動後の累積値です。

DoA/VADのI2CコマンドはSeeed Studio公式例にあるresource ID 20、read command `(19 | 0x80)`、4-byte payloadをそのまま実装しています。応答先頭のraw status byteは公式例に意味の定義がないため、成功値やbitを推測して判定に使いません。共有プロトコルの`xvf_status`はraw値ではなく、ESP32側で確認した通信・取得状態（STARTING/READY/ERROR）です。

## 前提

1. XVF3800へI2S対応ファームウェアを書き込みます。公式DoA/VAD例が対応を明記している `respeaker_xvf3800_i2s_dfu_firmware_v1.0.7.bin` を基準にしてください。
2. VS CodeのPlatformIO拡張機能を初期化するか、ESP-IDF 5.4以上6.0未満をインストールします。
3. 初回ビルド時はComponent Managerが `espressif/esp_tinyusb` を取得できるネットワーク接続が必要です。

XVF3800ファームウェアを書き込むときは3.5 mmジャック側のXVF3800 USB-Cを使います。XIAOアプリの書き込みとEK-RA8P1との実行時通信には、XIAO ESP32S3側のUSB-Cを使います。

公式資料:

- [ReSpeaker XVF3800 with XIAO 入門](https://wiki.seeedstudio.com/ja/respeaker_xvf3800_xiao_getting_started/)
- [XVF3800 DoA/VAD](https://wiki.seeedstudio.com/respeaker_xvf3800_xiao_doa_vad/)
- [16 kHz/stereo/32 bit I2S例](https://wiki.seeedstudio.com/respeaker_xvf3800_xiao_udp_audio_stream/)

## ビルドと書き込み

Wi-Fiを使う場合は、先に`main/app_config.h`の`APP_WIFI_SSID`、`APP_WIFI_PASSWORD`、`APP_UDP_DESTINATION_IPV4`を設定します。宛先IPv4はUDPを待ち受けるPCのアドレスです。空文字のままならWi-Fiを開始せず、USB音響機能だけが動作します。実際の認証情報はGitへcommitしないでください。PC側の待受はNmap Ncatなら`ncat -u -l 5005`、OpenBSD系netcatなら`nc -u -l 5005`です。

通常はリポジトリ直下のVS Codeワークスペースを開き、`ESP32: Build`または`ESP32: Upload`タスクを使います。PlatformIO設定は`platformio.ini`にあり、Espressif32 6.13.0、ESP-IDF 5.5.3、`seeed_xiao_esp32s3`を固定しています。全体の操作は[VS Code統合開発手順](../../docs/ra8p1/VSCODE_WORKFLOW.md)を参照してください。

ESP-IDFを直接使用する場合は、ESP-IDFターミナルでこのディレクトリへ移動して実行します。

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash
```

現行アプリはXIAO側USB-Cを実行時のTinyUSB CDC専用に使うため、実行中COMポートからROMダウンロードモードへ自動遷移できない場合があります。PlatformIOがCOMポートを検出した後、`Connecting...`と`Write timeout`で停止した場合は、次の手順で手動移行してから`ESP32: Upload`を再実行します。

1. EK-RA8P1 J7をXIAO側USB-Cから外し、XIAO側USB-CをPCへ直接接続します。
2. XIAO ESP32S3本体の`BOOT`を押したまま、XIAO本体の`RESET`を押して離します。
3. `BOOT`を離し、WindowsでCOMポートが列挙されたことを確認します。
4. `ESP32: Upload`を実行します。

ReSpeaker基板側のXVF3800用`RESET`ではESP32-S3をROMダウンロードモードへ移行できません。XIAO上の小さい`BOOT`/`RESET`を使用してください。ボタンを見分けにくい場合は、XIAO側USB-Cを外し、XIAOの`BOOT`を押したままPCへ接続してから`BOOT`を離す方法でも移行できます。

GPIO43/44はXVF3800のI2Sに使用するため、UART0 consoleは無効です。また、XIAOのUSB-Cは実行時にバイナリ専用CDCとして使うため、標準出力をUSBへ混在させていません。`idf.py monitor`へテキストログが出ないのは意図した設定です。

## 単体確認

1. XIAO側USB-CをPCへ接続してリセットします。
2. デバイスマネージャーでCDC COMポートが1個だけ列挙されることを確認します。
3. COMポートをバイナリで読み、先頭が `53 52 01 01` のHELLO、その後にtype `02` の観測が約20 Hz、type `03` のhealthが約1 Hzで届くことを確認します。
4. 無音時と発声時で `level_dbfs_x100`、`peak_dbfs_x100`、`vad`が変化することを確認します。
5. 音源を周囲へ移動し、有効なDoAが `0..359`、読出失敗時が `0xFFFF`になることを確認します。UDP診断では`xvf_raw_status`も併せて記録し、DoA/VAD固定時のXVF3800応答切り分けに使います。
6. CRC-16/CCITT-FALSEを検証し、sequenceが増加することを確認します。

フレーム境界はUSB転送境界と一致するとは限りません。受信側はmagic、payload length、CRCを使ってストリームから復元してください。

## EK-RA8P1との接続確認

1. EK-RA8P1 J7をUSB hostとして有効にし、VBUSENを含むUSB HSピン設定を生成します。
2. J7へUSB-C male to USB-A femaleのhost adapterを挿し、data対応USB-A to USB-CケーブルでXIAO側USB-Cへ接続します。
3. EK-RA8P1側を先に起動してからReSpeakerを接続します。
4. CPU0でHELLO受信、50 ms前後の観測更新、sequence、CRC error count、timeoutを確認します。
5. USBを抜いたときCPU0がアクチュエータを安全停止し、再接続後に新しいHELLOを受けて復帰することを確認します。

EK-RA8P1からのVBUS給電を使う場合も、モーター電源や3S LiPoをUSB 5 Vへ直結しないでください。電源容量、突入電流、GND経路は実機配線に合わせて確認します。

## コード構成

- `xvf3800_control`: 公式resource-based I2CコマンドによるDoA/VAD取得。I2S firmwareが`DOA_VALUE`を拒否した場合はAEC auto-selected beamへfallback
- `audio_capture`: I2S連続取得、RMS/peak dBFS、DMA overrun計数
- `usb_link`: TinyUSB単一CDCの列挙、マウント検出、双方向バイナリ転送
- `acoustic_frontend`: 取得値の統合、共有プロトコルの周期送信
- `wifi_telemetry`: Wi-Fi station、自動再接続、CPU0診断のUDP JSON変換
- `app_main`: 各責務の起動順序だけを管理
