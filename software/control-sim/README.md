# control-sim

`firmware/ra8p1/SoundExplorationRover_CPU0/src/control/` と、I/Oを持たない
`services/acoustic_identifier.c` の可搬なCコードをホストのclangで直接ビルドし、
Pythonから呼び出して回帰試験する基盤です。
Python側へ制御ロジックを再実装しないこと、C構造体のレイアウトを毎回検査することを
基本ルールにします。

## 実行

```sh
cd software/control-sim
uv run pytest -q
uv run python build.py --layout
uv run python build.py --sanitized-smoke
uv run python build.py --feature-protocol
```

`controlsim.replay` は `software/rover-monitor/rover-monitor.log` のような
logging prefix付きJSON Linesを読み、`sensors` 部分を実際のC回避コントローラへ流します。
壊れた行は既定でエラーにして、`strict=False` を指定した場合だけ明示的に無視します。

サニタイザ付き実行はPythonプロセスへ共有ライブラリをロードせず、同じCソースを
スタンドアロン実行します。macOSのframework PythonとASan dylibの非互換を避けるためです。

`--feature-protocol`は共有`0x04 ACOUSTIC_FEATURE`の符号化・ストリーム復号と、
CPU0の80フレーム再組立をASan/UBSan下で検証します。欠落、重複、イベント混線、
メタデータ不正時に不完全パッチが破棄されることも確認します。

## 前向き3眼ToFの壁回避

`controlsim/wall_world.py` は実際のC出力をToF・IMU入力へ戻す簡易閉ループモデルです。
前端突出・センサー位置・3:1平滑化・駆動応答を仮定し、速度・旋回性能・進入角・光軸のずれを変えて
接触と同じ動作の繰り返しを検出します。実機校正済みのシミュレータではありません。
`tests/fixtures/wall_loop_20260915.jsonl` は実機ログから制御関連値だけを保存した回帰入力です。
測定時刻を使って再生し、旧ログの軌跡が新しい指令でも再現されるとは仮定しません。

[原因・設定・検証結果](../../docs/firmware/validation/sensor-avoidance-20260915.md)を参照してください。
