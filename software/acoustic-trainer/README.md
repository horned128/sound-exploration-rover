# Acoustic Learning Reference

Sound Exploration Roverの音響特徴量と現場学習のホスト参照・検証環境。NumPyによる特徴量参照実装と、
背景AE＋能動フレーム照合の決定論的な参照実装を含む。音響認識はTensorFlow Lite Microを使わず、
CPU0の純C実装とこの数値契約だけで完結する。

## 環境

Python 3.12（`.python-version`）、依存は `uv.lock` で固定。
macOS Apple Siliconで検証。ホスト用コンパイルにはXcode Command Line ToolsのC/C++コンパイラが必要。

```sh
cd software/acoustic-trainer
uv sync --locked
uv run python tests/verify_vendor.py
uv run python tests/build_runtime.py
uv run python tests/smoke_test.py
uv run python tests/test_build_log.py
uv run python tests/test_log_mel.py
uv run python tests/test_acoustic_learning.py
```

`build_runtime.py` はリポジトリ内のベンダソースと**実際のCPU0 Cラッパー**をコンパイルする。
ホストのμT-Kernel基本型だけは既存control-simのshimを使用し、推論器は代替しない。
生成物は無視対象の `build/`、仮想環境は `.venv/` へ置く。

`smoke_test.py` はPhase 4で使う予定の制御MLPを固定乱数の合成データで1ステップ学習し、
入力 `[1,10]` / 出力 `[1,2]` の完全int8モデルへ変換する。
TFLite参照カーネルとTFLMの11テスト入力に対する出力を比較する（許容差1 LSB）。
未初期化、モデル破損／切詰め／不整列、float入出力、入出力サイズ不一致、
アリーナ不足、再初期化失敗、reset後の復帰も確認する。
学習APIを含むため、ライブラリimportだけのチェックではない。

結果は `build/smoke_report.json` と `build/smoke_int8.tflite` に出力する。
この検証は音響識別から独立している。

`test_build_log.py` はe² studioのClean-only完了／別プロジェクト完了を
本ビルド完了と誤認しないことを検証する。

`features.py` は固定数値契約どおりのlog-melホスト参照実装である。
ESP32S3と同じ単精度演算順を再現し、`test_log_mel.py` が無音、インパルス、
純音、混合音、固定疑似雑音と、256 samplesを含む不均一な投入境界で
最終32-bin int8出力のビット一致を検証する。倍精度FFTを正とせず、実機へ渡す
学習入力と組み込み出力が同じ量子化値になることを契約とする。

`acoustic_learning.py` はCPU0と共有する音響の現場学習の数値仕様である。seed固定の
32→16エンコーダ、RLSデコーダ、能動フレーム時の背景更新ゲート、能動フレームの
`mean[32], std[32], max[32]` int8要約、個別5見本へのcosine最小距離、leave-one-outの
受理しきい値（最大0.200）、代表ピーク周波数帯の±3 binゲートを実装する。`test_acoustic_learning.py`は回帰で、能動フレーム不足を
「非対象」ではなく「判定不能」とすることまで検証する。合成データで実録音の識別精度を
主張するものではない。

## 実装との対応

- 特徴量・通信・IPCの固定値は
  [エッジAI設計](../../docs/firmware/edgeai/README.md)に従う。
- 音響の背景AE・能動フレーム識別と保存形式は、`acoustic_learning.py`の数値契約を保った
  純Cサービスとして実装する。実録音のしきい値は実機でのみ確定する。
- CPU0の `tflm_runtime_get_info()` はPhase 4のMLP用に保持する。ホスト測定値はポインタ幅が
  異なるため、実機の使用量や実行時間を代用しない。
- 選択演算、モデル寿命、単一タスク条件は
  `firmware/ra8p1/common/tflm/README.md` を参照。
- `task_infer`の起床機構とESP32のイベント送信、背景AEのCPU0移植、32 KiB MRAM保存は実装済み。
  実機検証と走行経路への安全側の接続は未完である。

## TFLM基盤の検証記録（Phase 4用）

- TensorFlow 2.20.0 / NumPy 2.5.3。
- TFLMとCラッパー206ソースをホストでコンパイル。
- 合成int8制御MLPのTFLite参照出力との差0 LSB。
- ホストのアリーナ実使用量1,120 B。境界条件とアリーナ不足からの復帰も成功。
- ベンダ485ファイルのハッシュ一致。CヘッダのC99コンパイル成功。
- ビルドログ回帰3テスト、既存control-sim 5テスト成功。
- CPU0 Generate + Clean Buildおよび生成差分整理後のクリーン再ビルド成功。
  ELFのGNU sizeはtext 135,112 B / data 320 B / bss 123,848 B。
  96 KiBの静的アリーナと全公開APIのリンクを確認。
- ターゲットビルドには外部ヘッダとnewlib nosysの警告が残る。
  nosysはOS I/Oを実装せず失敗を返す。TFLMの診断文字列はCPU0で無効化している。
