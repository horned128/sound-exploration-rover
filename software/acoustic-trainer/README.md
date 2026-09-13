# Acoustic Trainer

Sound Exploration RoverのPhase 1用Python環境。NumPyによる特徴量参照実装、
TensorFlowによる埋め込みCNNの学習・完全int8 TFLite変換、Matplotlibによる評価に使用する。
学習データと本番の音響モデル・特徴量抽出はPhase 2で追加する。

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
```

`build_runtime.py` はリポジトリ内のベンダソースと**実際のCPU0 Cラッパー**をコンパイルする。
ホストのμT-Kernel基本型だけは既存control-simのshimを使用し、推論器は代替しない。
生成物は無視対象の `build/`、仮想環境は `.venv/` へ置く。

`smoke_test.py` は固定乱数の合成データで小さなCNNを1ステップ学習し、
入力 `[1,80,32,1]` / 出力 `[1,64]` の完全int8モデルへ変換する。
Conv2D・DepthwiseConv2D・AveragePool2D・Reshape・FullyConnectedを通し、
TFLite参照カーネルとTFLMの11入力に対する出力を比較する（許容差1 LSB）。
未初期化、モデル破損／切詰め／不整列、float入出力、入出力サイズ不一致、
アリーナ不足、再初期化失敗、reset後の復帰も確認する。
学習APIとMatplotlibの描画を含むため、ライブラリimportだけのチェックではない。

結果は `build/smoke_report.json`、検証モデルは `build/smoke_int8.tflite`、
描画確認は `build/smoke_embedding.png`。この合成モデルに音源識別能力はない。

`test_build_log.py` はe² studioのClean-only完了／別プロジェクト完了を
本ビルド完了と誤認しないことを検証する。

## Phase 2への引継ぎ

- 特徴量・通信・IPCの固定値は
  [エッジAI設計](../../docs/firmware/edgeai/README.md)に従う。
- int8入力でも、モデルのscale/zero pointと特徴量量子化の一致を確認する必要がある。
- CPU0の `tflm_runtime_get_info()` でアリーナ実使用量を取得できる。
  ホスト測定値はポインタ幅が異なるため、実機の使用量や実行時間を代用しない。
- 選択演算、モデル寿命、単一タスク条件は
  `firmware/ra8p1/common/tflm/README.md` を参照。
- Phase 1ではタスク登録や走行経路への接続を行わない。実機検証は不要。

## Phase 1検証記録（2026-09-11）

- TensorFlow 2.20.0 / NumPy 2.5.3 / Matplotlib 3.11.1。
- TFLMとCラッパー206ソースをホストでコンパイル。
- 合成int8モデル11,824 B、11入力の参照出力との差0 LSB。
- ホストのアリーナ実使用量6,992 B。境界条件とアリーナ不足からの復帰も成功。
- ベンダ485ファイルのハッシュ一致。CヘッダのC99コンパイル成功。
- ビルドログ回帰3テスト、既存control-sim 5テスト成功。
- CPU0 Generate + Clean Buildおよび生成差分整理後のクリーン再ビルド成功。
  ELFのGNU sizeはtext 135,112 B / data 320 B / bss 123,848 B。
  96 KiBの静的アリーナと全公開APIのリンクを確認。
- ターゲットビルドには外部ヘッダとnewlib nosysの警告が残る。
  nosysはOS I/Oを実装せず失敗を返す。TFLMの診断文字列はCPU0で無効化している。
