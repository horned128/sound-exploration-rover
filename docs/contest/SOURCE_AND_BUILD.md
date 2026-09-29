# プログラムの取得と再現

発送したSEROV本体には、提出時点のCPU0・CPU1・ESP32-S3ファームウェアを書き込み済みです。実機の動作確認は[実機評価手順](EVALUATION.md)から始めてください。以下はソースコードを読み、必要に応じて再ビルドするための案内です。

## ソースコード

リポジトリ: [sound-exploration-rover](https://github.com/horned128/sound-exploration-rover)

発送した実機で動作を確認したファームウェアのコミットは`4d71ffd686214556e60ac093fff3a84dbf75bf74`です。提出資料を加える前の版なので、今後の文書更新とは区別できます。GitとGit LFSを導入したPCで、同じ版を取得するには次の手順を使います。

```sh
git clone --recurse-submodules https://github.com/horned128/sound-exploration-rover.git
cd sound-exploration-rover
git checkout 4d71ffd686214556e60ac093fff3a84dbf75bf74
git submodule update --init --recursive
git lfs pull
```

μT-Kernel 3.0のBSPとPapaya Pathfinderの車体設計はGitサブモジュールです。独自の追加機構のSTEP/STLはGit LFSを使います。ZIPダウンロードではサブモジュールとLFSの実データがそろわないため、上記の方法で取得してください。

| 対象 | ソースと説明 |
|---|---|
| RA8P1 CPU0 | [`firmware/ra8p1/SoundExplorationRover_CPU0/`](../../firmware/ra8p1/SoundExplorationRover_CPU0/)、[設計書](../firmware/ARCHITECTURE.md) |
| RA8P1 CPU1 | [`firmware/ra8p1/SoundExplorationRover_CPU1/`](../../firmware/ra8p1/SoundExplorationRover_CPU1/)、[設計書](../firmware/ARCHITECTURE.md) |
| ReSpeakerをつなぐESP32-S3 | [`firmware/esp32s3/`](../../firmware/esp32s3/)、[README](../../firmware/esp32s3/README.md) |
| 障害物回避モデル | [`control_mlp_model.h`](../../firmware/ra8p1/SoundExplorationRover_CPU0/src/control/control_mlp_model.h)、[学習スクリプト](../../software/control-sim/controlsim/train_control_mlp.py) |
| 制御シミュレーション・回帰試験 | [`software/control-sim/`](../../software/control-sim/)、[README](../../software/control-sim/README.md) |
| ローバの機構 | [`hardware/`](../../hardware/) |

提出構成では、CPU0が音の識別と方位追従、距離センサ（ToF）と姿勢センサ（IMU）の取得、TensorFlow Lite Micro（TFLM）の制御モデル推論を担当します。CPU1はアクチュエータと通信途絶時の停止を担当します。両コアでμT-Kernel 3.0が動作します。音響埋め込みCNNによる識別と、三角測量による自動到着停止は、提出時の設定では無効です。現場学習した音は音響特徴量の見本照合で識別します。

設定の根拠は[`control_config.h`](../../firmware/ra8p1/SoundExplorationRover_CPU0/src/config/control_config.h)と[`task_config.h`](../../firmware/ra8p1/SoundExplorationRover_CPU0/src/config/task_config.h)です。

## 開発用PCでのビルド

RA8P1のCPU0・CPU1にはRenesas e² studio、FSP 6.4.0、対応するArm GNUツールチェーンとJ-Linkが必要です。ESP32-S3にはPlatformIO（Espressif32 6.13.0、ESP-IDF 5.5.3系）を使います。VS Codeの統合手順は[VS Code開発手順](../firmware/VSCODE_WORKFLOW.md)にあります。

1. リポジトリ直下の`SoundExplorationRover.code-workspace`をVS Codeで開く。
2. `RA8P1: Generate + Clean Build All`を実行し、CPU0とCPU1をビルドする。
3. `ESP32: Build`を実行する。

RA8P1を書き込み直す場合は、[CPU1、CPU0の順で書き込む手順](../firmware/VSCODE_WORKFLOW.md#3-ra8p1の書き込みとデバッグ)に従ってください。ESP32-S3の書き込み時にはXIAOのUSB-CをEK-RA8P1のJ7から外してPCへつなぎます。詳しくは[ESP32-S3 README](../../firmware/esp32s3/README.md#ビルドと書き込み)を参照してください。

## PC上での制御試験

`software/control-sim`はCPU0の可搬なC制御コードをホスト側でビルドして試験します。Python 3.11または3.12、`uv`、Clangが必要です。

```sh
cd software/control-sim
uv run pytest -q
uv run python build.py --layout
uv run python build.py --sanitized-smoke
```

プランター配置を模した試験は[`tests/test_test_map_scenario.py`](../../software/control-sim/tests/test_test_map_scenario.py)にあります。モデルの学習方法は[制御MLP設計書](../firmware/edgeai/control_mlp_avoidance.md)を参照してください。シミュレーションの結果は実機試験の代用としては扱っていません。

## ライセンス

独自のソースコードにはリポジトリ直下の[MIT License](../../LICENSE)を適用します。主な第三者資材のライセンス表記は次のとおりです。その他の同梱ライブラリや生成物は各ファイルの表示を参照してください。

| 資材 | 表記・参照先 |
|---|---|
| μT-Kernel 3.0 BSP2 | [BSP README](../../firmware/ra8p1/common/mtk3_bsp2/README.md)にT-License 2.2と記載 |
| Papaya Pathfinder | [Apache License 2.0](../../hardware/papaya-pathfinder/LICENSE) |
| TensorFlow Lite Micro | [Apache License 2.0](../../firmware/ra8p1/common/tflm/LICENSE)。同梱のthird_partyは各ディレクトリの表記も参照 |
| CMSIS | [同梱ライセンス](../../firmware/ra8p1/SoundExplorationRover_CPU0/ra/arm/CMSIS_6/LICENSE) |
| Bosch BMI270 SensorAPI設定 | [ルートREADMEの表示](../../README.md#サードパーティライセンス) |

Renesas FSPの生成物やそのほかの第三者コードについても、各ファイルのライセンス表示が優先します。
