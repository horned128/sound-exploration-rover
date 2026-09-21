# Acoustic AI Lab

`Acoustic AI Lab`は、音源識別の学習・推論品質を改善するためのローカル計測ツールです。既存の`software/rover-monitor`やESP32S3のUDPテレメトリは変更しません。

CPU0とPCをEK-RA8P1のJ11 USB Full-Speed端子で直接つなぎ、現在の推論値と学習状態を記録・可視化します。記録はJSON Linesなので、問題が起きた時刻のログをそのまま渡して、識別しきい値・特徴量・背景モデルを更新する根拠にできます。

## できること

- 4 Hzの推論snapshotを`logs/acoustic-ai-lab-*.jsonl`へ追記保存
- `TARGET / NOT TARGET`、cosine distanceとしきい値、peak bin、有効フレーム数、背景MSE、VAD/DOAをブラウザで表示
- 現在の192次元特徴量要約と、MRAMに保存された各見本の平均をグラフ表示
- ブラウザから「新規学習開始」「5見本をMRAMへ保存」「収集取消」「保存済み見本読出し」を実行

このツールが送れるのは上記の学習・照会コマンドだけです。車輪、操舵、走行モードを操作する機能はありません。

## 接続

1. CPU0をこの変更を含むファームウェアで書き込みます（このリポジトリでビルド済み。書込みは別途実施します）。
2. XIAO ESP32S3 / ReSpeakerは従来どおりJ7へ接続したままにします。
3. データ通信対応USBケーブルで、PCと**J11 USB FS**を直結します。J10（J-Linkデバッグ端子）は使いません。
4. モーター電源を含むローバの電源構成は従来の安全手順に従ってください。J11をPCへつなぐとUSB給電される場合があるため、電源を二重に投入しない構成を確認してから接続します。

macOSでは接続後のポートを確認できます。

```zsh
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
```

## 起動

```zsh
cd /Users/hino/Desktop/TRON2026/sound-exploration-rover/software/acoustic-ai-lab
uv run python main.py --port /dev/cu.usbmodemXXXX
```

ブラウザで <http://127.0.0.1:8010> を開きます。HTTPポートは`--http-port 8011`または`ACOUSTIC_AI_LAB_HTTP_PORT`で変更できます。

USBをまだ接続しないオフライン確認では、`--port`なしで起動できます。画面は開きますが、学習コマンドは送れません。

## ログを使った判定

`snapshot`レコードに、各推論時点の`cosine_distance`、`identifier_threshold`、`target_peak_bin`、`current_peak_bin`、`background_mse`、`background_threshold`、`infer_status`を保存します。誤検知・見逃しがあった前後のログと、そのとき鳴らした音を対応付けると、以下を分けて評価できます。

- 声と口笛の特徴距離が近すぎるか
- peak bin gateが音色差を十分に弾けているか
- 背景MSEが環境音を異常として捉えているか
- 学習5見本のばらつきが大きすぎないか

ログ中の`summary_chunk`と`profile_chunk`は、現在の特徴量と保存済み見本の192次元値です。これにより、同じ`TARGET`でもどの帯域特徴が似た扱いになったかを後から解析できます。

## 開発者向け確認

```zsh
cd /Users/hino/Desktop/TRON2026/sound-exploration-rover/software/acoustic-ai-lab
python -m unittest discover -s tests -v
```

`protocol.py`は外部ライブラリを使わず、ファームウェア共通の`acoustic_protocol`と同じCRC-16/CCITT-FALSEフレームを検証します。
