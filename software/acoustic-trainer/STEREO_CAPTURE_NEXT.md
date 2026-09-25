# 次の実機作業: XVF3800 I2S 第2スロットの確認

PC側だけで進めた18音種事前学習・未知3音種ずつの分離評価では、小型モデルは混合音の検出と低い誤検知を両立できなかった（[`FEWSHOT_FIELD_EVALUATION.md`](FEWSHOT_FIELD_EVALUATION.md)）。次は**いま利用可能な第2 I2S スロットに独立した情報があるか**だけを最小の実機収録で確かめる。TARGETの音種は固定しない。

## 既に確認済み

- 現行配線はXVF3800→XIAO ESP32-S3の16 kHz、stereo、32 bit I2S。`audio_capture.c` は左スロットだけをmonoへ変換していた。右スロットは一度もログに保存されていなかった。
- Seeedの[公式I2S例](https://wiki.seeedstudio.com/respeaker_xvf3800_xiao_udp_audio_stream/)は2スロットの処理済み出力を示すが、**4マイクの未処理信号が各スロットから取り出せるとは記していない**。右スロットが別ビーム、複製、または無音かは実機データなしでは分からない。
- CPU0に割り当てられたSRAMは936 KiB、Code MRAM/イメージ領域は512 KiB（`docs/firmware/ARCHITECTURE.md`、`firmware/ra8p1/README.md`）。アプリ分割領域1 MiBのESP32-S3ファームは現時点で約778 KiB。PC上の大型事前学習モデルをそのまま積む前に、縮小後の未知音・混合音評価が必要。

## 準備済みのファームと解析

- `firmware/esp32s3/src/app_config.h`: `APP_AUDIO_DATASET_STREAM_ENABLE=1U` と `APP_AUDIO_STEREO_DIAGNOSTIC_ENABLE=1U` の**収録用ビルド**。ESPのPCM先頭チャネルは従来どおり `acoustic_pcm`、第2チャネルは `acoustic_pcm_ch1` として同じsample番号付きUDP JSONLへ記録する。CPU0のUSB/features/モーター判定は変更しない。
- `software/rover-monitor/diagnostics.py`: 両チャネルをJSONL専用記録とし、WebSocketへ生PCMを流さない。
- `stereo_pcm_audit.py`: ESP再起動とUDP欠落を考慮して両チャネルをsample番号で照合。レベル、同一sample比率、相関、連続した2ch WAVを出力する。相関が低くてもそれだけでは**独立4マイク**と断定しない。
- ESP32-S3はPCM診断有効・第2チャネル有効の構成でPlatformIOビルド成功。RAM **80,436 / 327,680 B**。第2チャネル無効時 **72,244 B**。両構成のビルド成功、ホスト診断テスト成功。

## 人間に必要な最小作業

**XIAO ESP32-S3の収録用ファームを1回書き込み**、Rover Monitorを動かして、次の3区間を1本のログまたは3本のログに保存する。CPU0/CPU1やXVF3800のファームを書き換える必要はない。

1. TARGETなし・TVなしの静かな状態: 約5秒。
2. 任意のTARGET単独: 5回ほど、約10秒。
3. 同じTARGET + TV同時: 5回ほど、約10秒。TVが比較的目立つ音量。

モーターの第2スロットも比較する場合に限り、TARGETを止めてSW1短押しで5〜6秒のモーター走行を追加する。既存のモーターのみ録音は第1スロットにあるため、まずは上記3区間だけでも第2スロットの可否を判断できる。録音時間帯（または各ログ名）だけ教えてもらえば、PCM復元・チャネル有用性・TARGET+TVでの一般音種few-shot比較はAI側で実行する。

```sh
# リポジトリ直下。XIAO側USB-CをPCへ接続して書き込む。
bash .vscode/scripts/Invoke-PlatformIo.sh Upload
# Rover Monitorは従来どおり software/rover-monitor から uv run main.py
software/control-sim/.venv/bin/python software/acoustic-trainer/stereo_pcm_audit.py \
  software/rover-monitor/logs/rover-monitor-YYYYMMDD-HHMMSS.jsonl \
  --output /tmp/stereo-audit.json --wav-dir /tmp/stereo-wav
```

ログに `record_type=acoustic_pcm_ch1` が無ければ第2スロットの診断ファームが動いていないかUDPに欠落がある。第2スロットが無音・左の複製ならこの配線のまま空間分離はできない。別の処理済み出力なら、左右それぞれの混合音検出・誤検知を**別音種も含めて**検証してからファーム判定へ接続する。収録後、通常運用ではPCM/第2スロット診断フラグを0にしてビルドし直す。
