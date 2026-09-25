# 音響ベースライン改善: 第2 I2S スロット採用

## 採用の根拠

CPU0で実行中のC関数そのもの (`acoustic_identifier_summary_create` / `acoustic_identifier_summary_classify`) に、ESP32-S3の既存C log-melで再生成したステレオ実録音を入力。TARGET単独から**音の名前を使わず5例**を登録し、他ログは登録に使わず評価した。全80フレーム窓の第1スロット/第2スロットの実測比較：

| ログ区間 | 第1スロット | 第2スロット |
| --- | ---: | ---: |
| TARGETのみ | 20/21 | 20/21 |
| TARGET＋雑音 | **9/34** | **23/34** |
| 静音 | 0/16 | 0/16 |
| モーターのみ | 0/18 | 0/18 |

40フレームずつ移動した80フレーム窓でも TARGET＋雑音21/67→44/67。これは現場で現行方式より**多くの混合パッチを拾う**根拠。ただし人間の発音時刻はイベント単位で記録されておらず、Recallではない。負例は静音・motor約28.8秒だけで、TVのみの第2スロット、拍手・紛らわしい音を含まない。誤検知率の保証や任意音種の精度向上は主張しない。再現スクリプト `software/acoustic-trainer/eval_legacy_stereo.py`、結果 [`baseline_slot1_20260925.json`](baseline_slot1_20260925.json)。

## 実装

- ESP32-S3 `APP_AUDIO_FEATURE_CHANNEL_INDEX=1`: 2スロットI2Sの右側の処理済みPCMから**これまでと同じ**16k/400/512/hop160/32bin int8 log-melを生成。TARGET名に依存するモデルは作らない。USB特徴量payload、送信packet数、CPU1 IPC、sensor/motor制御は変えない。PCM診断を有効にする場合のJSON `channel=0/1` は物理I2S slot順を保つ。
- ESP HELLOに `ACOUSTIC_CAPABILITY_FEATURE_SLOT1` を追加、CPU0は必須条件にする。CPU0だけ先に書き込んだ間、旧ESPのslot0特徴量で走行や学習を行わない。
- CPU0 MRAMレコードのversionを4→5。古いslot0で登録した5見本を新slot1音声と誤照合しない。保存領域とpayloadサイズは変更なし。旧保存値は無効となり、**新たに5回登録**が必要。
- `CPU0_SOUND_REQUIRE_IDENTIFIER_MATCH=1` 時は見本が未登録でも一致を必須とする。登録済みでない間、他音のDoA/VADだけで発進しない。タイムアウト・ToF/IMU安全調停・CPU1駆動処理はそのまま。
- 高速化・新NNは今回の変更に含まれない。送信周期を急に上げるとDoA観測欠落を生じ得るため、**精度改善を先行**し従来の約1.28秒のコード上の遅延下限は残る。

## 実機に必要な操作（AI側build済み）

1. **CPU0を先に**書き込み: `bash .vscode/scripts/Invoke-RaBuild.sh --target CPU0 --flash`。USBを再接続するまで古いESPのHELLOが受理されないのは期待動作。
2. 次にXIAO ESP32-S3を書き込み: `bash .vscode/scripts/Invoke-PlatformIo.sh Upload`。EK-RA8P1 J7からXIAO側USBを外し、PCに接続してXIAOのROMブートローダへ移行する手順は [`firmware/esp32s3/README.md`](../../firmware/esp32s3/README.md#ビルドと書き込み) に従う。書き込み後J7へ戻す。
3. Rover Monitorで `usb.hello=1` かつ初回 `learning.storage_valid=0` を確認。**SW1を2秒長押しして登録開始**、TARGETを5回鳴らし `learning.samples=5` を確認、再度SW1を2秒長押しして保存 (`storage_valid=1`)。今回はMRAM v5なので以前の見本は使わない。
4. 「TARGETなしTV」「TARGET＋TV」「TARGET＋モーター」を各数秒確認し、CPU0再起動後も見本がv5で読めることを確認する。TARGETの種類は固定しない。ログを渡せば誤検知・見逃し・実時間をAIが再評価する。

AIエージェント環境には実機USB/J-Linkが列挙されておらず、上の書込み・発音・走行確認を代行できない。CPU0・ESP32-S3ビルド、host protocol/識別・センサー生存テストは通過。現在ワークツリーのPCM UDP/ステレオ診断/短押しモータ走行は通常運用向けにコンパイルOFF。再録音が必要なときだけ一時的に有効化する。
