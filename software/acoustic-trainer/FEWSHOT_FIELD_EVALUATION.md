# 未知音・混合音の追加評価（2026-09-25）

## 実機録音と再現

- `rover-monitor-20260925-075415.jsonl`: ESP再起動を含む2音声セッション。モーター指令と車輪エンコーダを同時に確認した区間を抽出すると約26.8秒。ログ全体を1つのWAVに連結しない。
- `rover-monitor-20260925-080028.jsonl`: 走行指令を除いたTARGETのみのPCM約51.3秒。学習対象の**名前**はモデルへ与えない。
- `rover-monitor-20260925-004412.jsonl`: 静音、TV、人声、TARGET+motor、TARGET+TV+motor。後半は断続発音の詳細な開始/終了が不明なためセッションレベルのラベル。
- `software/audio_ml/pcm_dataset.py` はESP再起動によるsample番号の再利用とpacket欠落を区別する。`labeled_recordings.py` は車輪実測のあるモーター区間のみをWAV化する。全PCMデータはgitへ追加しない。

```sh
python3 software/audio_ml/labeled_recordings.py \
  --motor-log software/rover-monitor/logs/rover-monitor-20260925-075415.jsonl \
  --target-log software/rover-monitor/logs/rover-monitor-20260925-080028.jsonl \
  --output /tmp/field-two
python3 software/audio_ml/field_session.py \
  software/rover-monitor/logs/rover-monitor-20260925-004412.jsonl --output /tmp/field-mixed
```

## 音種を固定しない方式比較

ESP32-S3の**実際のC log-mel**とCPU0の実際の192D見本生成を使用。対象音専用学習はしていない。現行保存見本の3/5合意を照合した20 frame/80 ms更新のA方式は、今回の現場負例約120秒では受理0、TARGETのみの区間で616/637窓が受理。これを一般化した実績とはせず、公開一般音響データの異音源/混合音でも検証した。

一般音響データ: ESC-10 10カテゴリ、ESC-50の24カテゴリから12録音ずつ。上流commit固定 `33c8ce9eb2cf0b1c2f8bcf322eb349b6be34dbb6`。後者はCC BY-NC 3.0なので音声を配布・commitしない。18カテゴリでTARGET非依存のmixture metric学習、3カテゴリで選定、残る3カテゴリ（clapping/church_bells/glass_breaking）で最終評価。評価は同一録音音源を5回登録しPCMで他音と混ぜる**正例が確実に存在**する設定と、同一カテゴリの別音源を含む負例。発音時刻の分からないESCカテゴリ判定accuracyとは区別する。

| 方式 | 独立カテゴリ/異音源での0 dB混合検出 | 負例の誤検知 | 判断 |
| --- | ---: | ---: | --- |
| 200 ms/96D手作り特徴量, 距離0.08 | validation 21/57、test 36/60 | validation 162 FP/h、test 396 FP/h | 低FP条件を満たさない |
| 48D時系列pool付metric CNN（約9.8kパラメータ） | validation 4/57 | validation 8.42 FP/h | 混合音の見逃しが多すぎる |
| query-conditioned CNN（約6.9kパラメータ） | 低FP閾値でvalidation 1/51、test 0/57 | 低FP閾値で0件 | 高速でも音源をほぼ検出できない |

ペア判定だけの学習内検証では query-conditioned CNN の見かけのprecisionは高かったが、**未知カテゴリ/未知音源**では検証側FPR 0.00067からテスト側0.01へ悪化した。走行中のfalse positiveを抑える閾値にすると混合音Recallがほぼ0になる。モデルの外部パラメータを量子化してファームへ配布する判断はできない。実機の録音だけで誤検知0を見た手作り閾値も、類似音など別データでは数百FP/hだった。数字はソースごとのアルゴリズム比較であり、実走行の発音からのP50/P95ではない。

現行log-melは**各10msフレームのlog-energy平均を引く**ため、音量を伴う手がかりが消え、強い背景音の重畳後にはTARGETの相対スペクトルも変わる。小型モデルの単純な縮小/閾値変更でこの情報を復元できない。400ms窓を用いたCNN案は2-of-3投票だけでも更新間隔80ms×2が加わるため、音の先頭からの判定は最低約560ms＋収集/通信/推論となり、今回の低遅延目標にも届かない。一方200ms窓の手作り案は速くできても、未登録音への誤検知が多い。量子化劣化やTFLM負荷を試す以前に浮動小数点host評価で落選した。

コード/結果: `software/acoustic-trainer/{fetch_general_audio.py,esc10_fewshot_eval.py,train_fewshot.py,eval_fewshot.py,eval_query_mixture.py,train_query_detector.py,eval_query_detector.py,eval_query_stream.py}`。機械可読要約 [`fewshot_evaluation_20260925.json`](fewshot_evaluation_20260925.json)。学習・検証音声、結果詳細JSONは同スクリプトの`--output`を指定して再生成する。

次の手順で一般音響データを取得・学習・別音種の評価を再実行できる（Python環境は`software/acoustic-trainer/.venv`、変数`TMP`は書込可能な一時ディレクトリ）。

```sh
PY=software/acoustic-trainer/.venv/bin/python
TMP=/var/folders/__/d9pz1vws43n9zs95zsqwyf540000gp/T/opencode
$PY software/acoustic-trainer/fetch_general_audio.py --output "$TMP/esc50-selected"
$PY software/acoustic-trainer/esc10_fewshot_eval.py --manifest "$TMP/esc50-selected/manifest.json" --output "$TMP/class-baseline"
$PY software/acoustic-trainer/train_fewshot.py --manifest "$TMP/esc50-selected/manifest.json" --output "$TMP/pooled-source-models" --examples 4800 --epochs 15 --task source --temporal-pool
$PY software/acoustic-trainer/eval_query_mixture.py --esc "$TMP/esc50-selected/manifest.json" --motor /tmp/opencode/labeled-two/wav/manifest.json --tv /tmp/opencode/field-labelled/wav/manifest.json --output "$TMP/query-mix"
$PY software/acoustic-trainer/train_query_detector.py --manifest "$TMP/esc50-selected/manifest.json" --output "$TMP/query-detectors" --pairs 5000 --epochs 10
$PY software/acoustic-trainer/eval_query_detector.py --manifest "$TMP/esc50-selected/manifest.json" --models "$TMP/query-detectors" --output "$TMP/query-detector-eval" --pairs 3000
$PY software/acoustic-trainer/eval_query_stream.py --esc "$TMP/esc50-selected/manifest.json" --motor /tmp/opencode/labeled-two/wav/manifest.json --tv /tmp/opencode/field-labelled/wav/manifest.json --target /tmp/opencode/labeled-two/wav/manifest.json --model "$TMP/query-detectors/query_12.keras" --output "$TMP/query-stream-eval"
```

上記の`/tmp/opencode/labeled-two`と`/tmp/opencode/field-labelled`は最初のPCM切出しコマンドで生成した出力先に合わせて指定する。macOSの別ユーザーでは`TMP`をそのユーザーの書込可能なパスに変更する。

**現時点で新検出器を走行判定に採用するGO条件は成立しなかった。** 既存の見本品質改善とPCM収録経路は維持する。次の方式として一般データを大幅に増やした混合音query学習、もしくはframeごとの絶対エネルギー/複数マイクの空間情報を失わないfrontendと、機器上のCPU/RAMを改めて設計する必要がある。TARGETを大量録音する解決策にはしない。

## 追加試験: 音量情報を残した新frontend（依頼後に実施）

ESP32-S3の既存32-bin特徴量は変更せず、`log_mel_extractor_process_window_pair()`を追加。従来のframe平均減算後log-melに加え、絶対log-energyを `clip(round((ln(mel_energy)+8)/0.125), -128, 127)` として**同じFFTから**出力できる。追加APIを使わない現行走行・通信・MRAM見本は同じまま。PCの量子化契約との一致は独立したC/NumPyテストで検証済み（従来centered bit一致、absolute許容1 LSB）。この絶対エネルギー版はまだUSBで送信せず、新認識器に切り替えていない。

TARGETに依存しないESC-50の18音種から、PCMで実際に背景音を重畳した正負ペアを作成。次の3+3音種と実機ログは学習から除外し、独立評価した。閾値は3音種の検証側だけで選び、別の3音種と現場データはholdoutした。

| 方式（400ms context + 80ms更新） | 0dB混合、検証3音種 | 0dB混合、未知テスト3音種 | FP/h（検証→テスト） | 現場TARGET+motor |
| --- | ---: | ---: | ---: | --- |
| absolute-mel query NN、6,869パラメータ | 8/54 | 7/63 | 6.67 → 5.71 | 検出できず |
| absolute-mel query NN、14,249パラメータ | 12/54 | 26/63 | 6.67 → **51.43** | 検出できず |
| PCMの最初200msから背景パワーを凍結・帯域別減算し短窓照合 | 3/63 | 4/63 | 1.90 → 1.90 | 別途実機未照合 |

大きいモデルは0dB混合の検出を改善したが、別音源では誤検知が激増した。小さいモデル/背景減算は低誤検知でも見逃しが大きい。40frameのCNN案は400msの音響context＋80ms×2の投票だけで560msを要し、通信と推論時間を含む要求500ms以下を満たせない。このため**量子化→TFLM→走行判断への接続は未実施**。追加録音が不足したのではなく、同じ固定データでの未知音・混合音の汎化が不足している。

絶対エネルギー/比較の再実行（`TMP`と録音manifestは上記の収録手順で用意）：

```sh
$PY software/acoustic-trainer/train_query_detector.py --manifest "$TMP/esc50-selected/manifest.json" --output "$TMP/query-absolute" --pairs 7000 --epochs 15 --frontend absolute --channels 8 12
$PY software/acoustic-trainer/eval_query_detector.py --manifest "$TMP/esc50-selected/manifest.json" --models "$TMP/query-absolute" --output "$TMP/query-absolute-eval" --pairs 3000 --frontend absolute
$PY software/acoustic-trainer/train_query_detector.py --manifest "$TMP/esc50-selected/manifest.json" --output "$TMP/query-abs-bigger" --pairs 12000 --epochs 20 --frontend absolute --channels 24
$PY software/acoustic-trainer/eval_query_stream.py --esc "$TMP/esc50-selected/manifest.json" --motor /tmp/opencode/labeled-two/wav/manifest.json --tv /tmp/opencode/field-labelled/wav/manifest.json --target /tmp/opencode/labeled-two/wav/manifest.json --model "$TMP/query-abs-bigger/query_abs_24.keras" --frontend absolute --output "$TMP/query-abs-bigger-stream"
$PY software/acoustic-trainer/eval_background_subtraction.py --esc "$TMP/esc50-selected/manifest.json" --motor /tmp/opencode/labeled-two/wav/manifest.json --tv /tmp/opencode/field-labelled/wav/manifest.json --output "$TMP/subtraction-eval"
```

次のハードウェア経路は、既存16 kHz stereo I2Sの**未記録だった第2スロット**が独立した処理済み音声かを先に確かめること。両スロットの収録ファームと検証スクリプトは [`STEREO_CAPTURE_NEXT.md`](STEREO_CAPTURE_NEXT.md) に記載した。一般音響事前学習はPCで継続できるが、CPU0イメージ512 KiB・ESP32-S3アプリpartition 1 MiBの制約から、大型事前学習モデルをそのまま走行時に積む解ではない。蒸留後の小型モデルも未知音・同時発音・低FP/hを独立検証して初めて採用する。
