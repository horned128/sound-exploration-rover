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
