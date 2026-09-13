# control-sim

`firmware/ra8p1/SoundExplorationRover_CPU0/src/control/` の可搬なCコードを
ホストのclangで直接ビルドし、Pythonから呼び出して回帰試験する基盤です。
Python側へ制御ロジックを再実装しないこと、C構造体のレイアウトを毎回検査することを
基本ルールにします。

## 実行

```sh
cd software/control-sim
uv run pytest -q
uv run python build.py --layout
uv run python build.py --sanitized-smoke
```

`controlsim.replay` は `software/rover-monitor/rover-monitor.log` のような
logging prefix付きJSON Linesを読み、`sensors` 部分を実際のC回避コントローラへ流します。
壊れた行は既定でエラーにして、`strict=False` を指定した場合だけ明示的に無視します。

サニタイザ付き実行はPythonプロセスへ共有ライブラリをロードせず、同じCソースを
スタンドアロン実行します。macOSのframework PythonとASan dylibの非互換を避けるためです。
