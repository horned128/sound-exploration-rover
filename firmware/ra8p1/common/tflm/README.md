# TensorFlow Lite Micro ベンダツリー

Phase 1でCPU0へ導入した、CMSIS-NNを使わない参照カーネル版。
アプリからは `SoundExplorationRover_CPU0/src/ai/tflm_runtime.h` のC APIのみを使用する。
CPU1とESP32S3にはリンクしない。

## 取得元と固定版

- [tensorflow/tflite-micro](https://github.com/tensorflow/tflite-micro/tree/e4e81fbdc3dd165b5345fb9580ce064fe650a766)
- commit: `e4e81fbdc3dd165b5345fb9580ce064fe650a766`（2026-09-11取得）
- [公式エクスポータ](https://github.com/tensorflow/tflite-micro/blob/e4e81fbdc3dd165b5345fb9580ce064fe650a766/tensorflow/lite/micro/tools/project_generation/create_tflm_tree.py)
  の既定ターゲット／参照カーネル出力。`.cc`の改名や独自のソース修正なし。
- FlatBuffers **25.9.23**、gemmlowp **719139ce755a0f31cbf1c37f7f98adcc7fc9f425**、
  ruy **d37128311b445e758136b8602d1bbd2a755e115d**、kissfft **v130**。
- 依存URLとチェックサムは `upstream/third_party_downloads.inc`、公式の依存パッチは
  `upstream/flatbuffers.patch` と `upstream/kissfft.patch` に保存。
  FlatBuffers/kissfftは公式ダウンロード処理によるパッチ適用済み。
- ライセンスはルートの `LICENSE` と各 `third_party/*/LICENSE` / `COPYING`。
  `SHA256SUMS.json` は取り込んだ全ソース、ヘッダ、ライセンス、取得手順のSHA-256。

ベンダファイルはCファームウェア規約の型・リンケージ・整形・lint対象から除外する。
更新時も公式ソースの一括更新とホスト／CPU0検証を行い、手作業で整形しない。

## 再取得

新しい一時ディレクトリへcloneし、上記commitへcheckoutする。
`software/acoustic-trainer/.venv/bin`（NumPyが必要）とGNU Make **3.82以上**のディレクトリを
PATHへ追加し、cloneのルートで以下を実行する。macOS標準Make 3.81は不可。
e² studio同梱 `com.renesas.ide.exttools.gnumake.*/mk/make` が使用できる。

```sh
python tensorflow/lite/micro/tools/project_generation/create_tflm_tree.py /tmp/tflm-export
```

出力の `tensorflow/` と `third_party/` をこのディレクトリへコピーする。
エクスポータはライセンスを収集しないため、ルートLICENSEとdownloads配下の各依存の
LICENSE/COPYINGもコピーする。上記 `upstream/` 記録とSHA-256を更新する。
`upstream/` はe² studioのソース探索から除外済み。

## CPU0のビルドと使用条件

`.project`のlinked resource `tflm` とDebug/Release両方の `.cproject` を設定済み。
C++17、hard-float、既存のM85指定とO2を使用し、例外／RTTI／スレッドセーフstatic初期化を無効化する。
`TF_LITE_STATIC_MEMORY` をTFLMとCラッパーで統一する。
テンソルアリーナは **96 KiB、16 byte境界、静的配列**。推論器と演算表もplacement newで構築し、
グローバルC++コンストラクタの実行に依存しない。公開APIをリンクのrootとして保持するので、
Phase 1でも実際の推論経路とアリーナがCPU0 ELFへ入り、リンク・メモリ収容を検証できる。

登録演算はConv2D、DepthwiseConv2D、FullyConnected、AveragePool2D、MaxPool2D、
Reshape、Softmax、Mean。Phase 2のモデル確定時に必要演算とアリーナ実使用量を再確認する。
Helium向け専用カーネルやCMSIS-NNの高速化効果は主張しない。実時間は実機で測定する。

1. 16 byte境界に配置したモデルの先頭と実バイト数を `tflm_runtime_init()` へ渡す。
2. `tflm_runtime_get_info()` で入出力サイズ、量子化scale/zero point、アリーナ使用量を取得する。
3. モデルの量子化仕様に従うint8入力を `tflm_runtime_invoke()` へ渡す。
4. 破棄は `tflm_runtime_reset()`。再初期化失敗時は旧モデルも無効になる。

モデルはビルド時に用意した信頼できる静的なTFLiteモデルを使用し、resetまで読み取り可能な
不変領域に保持する。FlatBuffersの境界／schemaと単一int8入出力を検査するが、
任意の外部モデルを安全に実行するサンドボックスではない。
全APIは**単一タスク専用、並行呼出し禁止**。モデルは入力1つ・出力1つ・subgraph1つに限定。
未初期化やサイズ不一致、割当失敗を結果コードで返す。アプリの停止やタスク生成は行わない。
CPU0は `NDEBUG` / `TF_LITE_STRIP_ERROR_STRINGS` で診断文字列を省き、結果コードで失敗を返す。
C++のnewlib依存は `--specs=nosys.specs` で解決する（ホスト用OS I/Oは使用しない）。
プロファイラ時刻は未接続（0）。実機計測はPhase 2で追加する。

```sh
bash .vscode/scripts/Invoke-RaBuild.sh --target CPU0 --regenerate --clean
```

ソース追加時は必ずGenerate + Clean Buildを実行する。
学習／ホスト検証手順は `software/acoustic-trainer/README.md` を参照。
