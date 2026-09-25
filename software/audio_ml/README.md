# 未知TARGETの混合音検出: 実装状況と再現手順 (2026-09-25)

## CURRENT_IMPLEMENTATION

- `firmware/esp32s3/src/app_config.h:19-37`: stereo I2S 16 kHz / 32 bit、256 samples/block (16 ms)、DoA観測20 ms、音量トリガ -45 dBFS、30 frame pre + 50 frame post、再トリガ1000 ms、2 frame packet 20 ms間隔。
- `firmware/esp32s3/src/log_mel_extractor.h:12-18` と `log_mel_extractor.c:189-243`: mono 400-sample Hann、512 FFT、160-sample hop (10 ms)、HTK 32 mel、**各frameのlog-energy平均を減算してからint8化**。frame全体のエネルギーと位相は保存されない。
- `firmware/esp32s3/src/audio_capture.c:75-91,199-223`: 80-frameリングの直近300 msを取得、残り500 msを待つ。`acoustic_frontend.c:301-336,436-465` で完成イベントを2 frameずつUSB CDCへ送る。
- `firmware/common/acoustic_protocol.h:12-30,163-170`: version 2、最大96 byte payload、32 bin × 2 frame/int8、80-frameイベント。`firmware/ra8p1/SoundExplorationRover_CPU0/src/services/acoustic_feature_assembler.c:26-63` は全40packetが揃うまで結果を公開しない。
- CPU0 `src/services/acoustic_identifier.c:44-139,184-258,341-489`: 80 frameを前後40 frameへ分割、各slotの能動frame (`std > 12`) の mean/std/max で192D。登録は5件 (`task_think.c:330-475`)、MRAM A/B保存 (`prototype_storage.c:10-42,260-321`)、重み付きcosine最近傍 + peak±6 bin、実効閾値最大0.055。`task_infer.c:307-364` は完成イベントごとに推論。音響照合にTFLite Microは**使用していない**（制御用TFLMは別）。

### 現行レイテンシのコード上の下限と計測点

音量トリガ後、post 500 ms + 全40 packet × 20 ms（初回即時なので最終送信まで約780 ms）+ USB/CPU0で**約1.28秒以上**。20 msトリガpoll、初回PCM窓25 ms、USB poll 1 ms、CPU0 think 50 ms、推論時間は追加。継続音のイベント開始はさらに最大1秒待つことがある。実測P50/P95は音発生時刻のラベルがなく算出不可。CPU0 Live Watch `g_task_acoustic_link_feature_transport_ms` は最初/最後のpacket間隔、`g_task_infer_processing_last_ms` / `max_ms` はCPU0処理時間（単調時刻の1 ms分解能）。発音起点のP50/P95ではない。

## FEASIBILITY_DECISION: CONDITIONAL GO

未知TARGETの少数見本による query-by-example **polyphonic sound event detection** / few-shot SED が最も近い。open-setは未知クラス受理の条件、prototypical・Siamese・contrastiveは学習方法であり、単なる事前定義クラス分類ではない。混合音に対する一般化は学習済み埋め込みでも保証されない。

既存の17ログには192D診断864件、現場profile、走行状態はあるが、WAV/PCM/raw log-melと、TARGETが実際に存在する区間・TARGETなしの区間を独立に記した正解ラベルがない。192Dから元音声や同時発音を復元できない。特にfrontendはframeごとにlog-energy平均を引くため、192Dやint8 log-mel同士の単純加算によるmixture augmentationは物理的な混合を再現しない。無検証のNNを自律走行判定へ入れればfalse positiveを評価できない。現在のログだけを根拠に「高速かつ混合音で低誤検知」を達成済みとすることは不可能。方式そのものは成立し得るため、最小限の実機音声収集を可能にして停止した。

## SELECTED_ARCHITECTURE（評価後の導入候補）

ESP32-S3は現行PCM→log-melを維持、CPU0で**200〜500 ms文脈 / 50〜100 ms更新**の小型埋め込みまたはquery-conditioned detectorと現場prototypeの判定を比較する。最初は異なる音種を分離した一般データでclass-disjoint episodic training、*PCMレベル*で背景を混合し、TARGETに依存しない同音/異音の対照学習を候補とする。混合音のbackgroundがTARGETより大きいとマスクされるため、低SNR限界を測る。短い衝撃・連続・断続の各イベントで評価する。

| 候補 | 判断 |
| --- | --- |
| A: 現行192Dの短窓/部分照合/背景正規化/N-of-M/PCEN | 最小資源。ただし192Dには時刻・PCMが残らず部分窓を現ログで再計算できない。ピークゲート除去や閾値拡大は未確認の誤検知を増やし得る。 |
| B: 事前学習CNNのcosine prototype | 小型で登録容易だが、TARGET+TVをTARGET単独へ近づける保証なし。 |
| C: mixture学習済みmetric/prototypical | 第一候補。未知クラスをholdoutした混合評価と実機量子化検証が必要。 |
| D: query-conditioned局所SED | Cで混合音に埋もれる場合に比較。フレーム単位の照合は時間平均の希釈を避ける。計算・RAMが増える。 |

ESPでembeddingを計算する案はUSB帯域を節約するが、ESPモデル実行と学習入力一致の検証が増え、RAへ送るprototype/現行MRAMとの整合が必要。CPU0側案は既存ESP DSPを維持できる。ただし現行80 frame/40 packet完了契約では100 ms更新にならない。選定モデルの入力と遅延の評価後に別の連続packet契約を設計し、旧ファーム同士を混在させない。source separationは必須と決めつけず、SNR別評価で判断する。

## IMPLEMENTATION / TRAINING_PIPELINE

- `legacy_audit.py`: 既存ログから現場見本、192D診断、走行proxyを抽出。4-of-5孤立見本、peak重み、閾値を再現。`--output` に全イベントCSVとファイル別JSON。**現行判定を正解ラベルとして使用しない**。
- `app_config.h` の `APP_AUDIO_DATASET_STREAM_ENABLE` を **収録時だけ1** にしてビルドすると、ESP32からI2S monoのPCM16LE (16 kHz) をbase64の `acoustic_pcm` UDP診断としてRover Monitor JSONLへ送信。256sample/packet、16packet bounded queue、drop数とfirst_sampleを添付。既存USB protocol・CPU0↔CPU1 IPCは変更しない。今回の収録用作業ツリーでは1、収録後は0へ戻す。Wi-Fi未接続やpacket欠落時は可聴音を捏造せず欠損を記録する。保存先JSONLは既存のmonitor logsで、Web画面への配信は抑止。
- `pcm_dataset.py`: PCM診断を検証し、欠損区間で分割してWAVへ変換。JSON annotationがあればclip/label/sessionをmanifestへ出力。欠損を跨ぐ注釈は拒否。データのラベルを自動推定しない。大量のTARGET+背景を個別録音する必要はない。
- この時点では一般音響データ／分離済み未知クラスのPCMと検証用混合・正解がなく、**学習、閾値選択、int8量子化、C配列化、TFLM音響モデル、streaming照合のファーム統合は未実施**。モデルファイルや精度を架空に作らない。

```sh
python3 software/audio_ml/legacy_audit.py --output /tmp/audio-audit
python3 software/audio_ml/test_pipeline.py
# PCM収録を有効にしたESP32-S3をビルド/書込み、rover-monitorで収録した後:
python3 software/audio_ml/pcm_dataset.py software/rover-monitor/logs/rover-monitor-YYYYMMDD-HHMMSS.jsonl --output software/audio_ml/dataset/field
```

注釈JSON例（`--annotations annotations.json`）: `{"rover-monitor-YYYYMMDD-HHMMSS.jsonl":[{"start_s":12.0,"end_s":14.0,"label":"target","session":"session1"}]}`。時刻はESP起動からの秒（`first_sample/16000`）、音声の入っていない部分をTARGETと呼ばないこと。WAVには未注釈の場合 `label:null` が付き、学習/評価の正例・負例にはしない。

## EVALUATION

機械可読集計 `evaluation/legacy_20260924.json`、全イベントCSVは上のコマンドで生成。17ログの診断864件、profileが一意に揃うセッションの500件を再照合し**499件一致**（1件はprofile更新時点や時刻不一致の可能性があり、正解ラベルではない）。20:22ログでモーター指令中60件の旧判定はTARGET 34 / NOT_TARGET 26。鳴っていない区間を含み、TPR/FNRではない。ピーク除去 + 閾値0.055の追加受理10件、閾値0.08で29件（別音かもしれない）。TV、人声、impact、類似音、motor-onlyの独立正解はなし。

| 指標 | 現行 | 新方式 |
| --- | --- | --- |
| Recall / Precision / FP/hour | 正解ラベルなしで算出不可 | 未学習で算出不可 |
| 発音→判定P50/P95 | 発音timestampなし | 未計測 |
| コード上の最短待ち | 約1.28秒 + 処理 | 実装・評価後に確定 |
| 音響モデル | 192D × 5 = 960 byte見本（MRAM背景モデル別） | 未選定 |
| inference時間/RAM | CPU0新設のLive Watchで実機計測待ち | 未選定 |

## BUILD_AND_TEST

`python3 software/audio_ml/test_pipeline.py` 2件成功（実測ログの再現、packet欠落時のWAV再構築拒否）。control-sim音響/通信9件成功。ESP32-S3 PlatformIOはPCM収録 **有効時** RAM 72,244 / 327,680 B、flash 777,645 B / 1,048,576 B、**無効時** RAM 62,820 B、flash 776,781 B、双方成功。CPU0 fast build成功（text 163,168 B, bss 221,040 B）。rover-monitor診断route 2件成功。control-sim全体: 220 pass / 54 fail（既存の `tests/test_control_mlp.py` がcallbackを2引数定義し、`controlsim/wall_world.py:90`が3引数で呼ぶ制御シミュレータの不整合；本変更箇所とは独立）。

## REMAINING_HARDWARE_VALIDATION

1. `APP_AUDIO_DATASET_STREAM_ENABLE=1` でESPファームをビルド・書込み、Wi-Fiとrover-monitorでJSONL収録。数十秒の背景（停止/走行、TV、人声等）と、未知TARGETを**5〜10回**（可能なら静かな状態で約10秒）だけ鳴らす。TARGET単独と、自然に重なる環境を各数秒ずつ収録すれば済む。手動の100回組合せ収録は不要。
2. `pcm_dataset.py` でblock欠落・sample_countを確認しWAV化。TARGETの開始/終了、背景のみの区間を短いannotationで指定。見本に混合区間を含めない。収録後はdiagnosticを0に戻す。
3. class-disjoint一般音響コーパス＋PCM混合でA/C/Dを比較し、閾値は別sessionのTV/voice/motor/impact/類似音でFP/hourを満たすものだけ採用。混合SNR別TPR/precisionとP50/P95を記録してから、選定モデルのint8/TFLMとstreaming USB契約を実装し、CPU0上の時間/RAMと実走行を確認する。

## 2026-09-25 実機ログ追記（提供された時刻ラベル）

新しい `rover-monitor-20260925-004412.jsonl` にはPCM 19,724 packet、冒頭の16 ms欠損を除いて約315秒の連続音声、および保存済み5見本が含まれていた。`field_session.py` はホスト時刻/ESP uptime/PCM sample番号を照合（clock offsetのP95散らばり15/24 ms）、0:45:50〜0:46:30静か、0:46:40〜0:47:05 TVだけ、0:47:30〜0:48:00人声、0:48:00〜0:49:00 TARGET+モータ、0:49:00〜0:51:00 TARGET+TV+モータを自動切出しする。モータが動いたのはTARGET検知中に限るので、**motor onlyのnegativeはこのログにはない**。後半2区間にはTARGETを鳴らした時間が含まれるが、断続発音の開始/終了は未知なので `target_possible` とし、窓ごとの真値として使わない。登録音は今回の測定対象であって、認識器に音種を固定しない。

既存192Dと同じ5見本による判定は158/158件再現。旧方式はTVのみ16件・人声のみ19件で受理0、TARGET+モータ60秒では14 TARGET / 23 OTHER / 1判定不能、TARGET+TV+モータ120秒では21 TARGET / 56 OTHER。これは**発音継続が保証されずRecallではない**。PCMから実際のCフロントエンドを用いた短窓A方式（20/32/40frame、80 ms更新、32D mean対96D mean/std/max）の候補を `short_window_probe.py` で再生した。200 ms・96Dは静音/TV/人声の同一録音から求めた負例最小距離0.08649未満で負例窓0/1081、TARGETがあり得る混合区間は586/1454窓が受理。負例を区間単位で1つずつholdoutすると人声で単発1窓の誤一致があるが2-of-3の投票で0。これは**同じ場所/同じ音の閾値探索**であり、未知音やmotor only・類似音の誤検知を保証しない。結果は `evaluation/field_20260925.json`、詳細window CSVと比較JSONは次で再生成する。

```sh
python3 software/audio_ml/field_session.py software/rover-monitor/logs/rover-monitor-20260925-004412.jsonl --output /tmp/field-labelled
software/acoustic-trainer/.venv/bin/python software/audio_ml/short_window_probe.py software/rover-monitor/logs/rover-monitor-20260925-004412.jsonl --manifest /tmp/field-labelled/wav/manifest.json --output /tmp/short-probe
```

PCM区間を活用して混合時短窓照合まで試したが、**TARGET単独PCMの現場登録再現と、モータだけの負例がない**。したがって短窓方式の低FPを検証済みとせず、未選定NNを量子化して走行判定に差し込むこともしない。追加の現場作業が必要なら、静かな状態でTARGETを5回鳴らす10秒程度のPCMと、音源なしでモータが回る10秒程度のPCMを別セッションで確保すれば、既存TV/人声/混合音と組み合わせられる。動作可能な実機においてmotor onlyを安全に発生できない場合は外部モータ録音を背景として使用し、その差は評価時に明記する。

### 保存済み見本の3/5合意の修正

今回のprofileは5件中3件が一群、2件が別群で、旧実装の「4件と異なる孤立1件」には該当しなかった。音の名前や周波数を固定せず、**4/5合意を優先し、なければ唯一の3/5合意で残り2件が全員から離れている場合だけ** 2件を取り直すようCPU0を修正した。保存済みprofileは読み込み時の照合対象にも同じmaskを適用する。曖昧な合意では5件全件を維持する。登録完了まで必要なら最大2イベント増え、MRAM/protocol形式は変更なし。

現行5件を使った compiled CPU0 C の再照合はmask `0b11100`（見本2,3,4）で、158件の旧判定に変更は**0件**。つまりこの修正だけで混合音の検出率が上がったとは主張しない。20frame/80ms更新の短窓実験は3件のみの見本、固定距離0.08で静音・TV・人声の合計1,081窓の受理0、TARGETが鳴る可能性のあるモータ併用区間345/678窓、TV・モータ併用区間566/1,454窓を受理した。80 msずつ重なる窓は独立試行ではなく、実機へこの閾値をまだ適用していない。陰性約93秒の0件だけでは、単純なPoisson 95%上限は**約116 FP/hour**で、求める低誤検知の証明にならない。

```sh
python3 software/control-sim/build.py  # compiled CPU0 classifier for field_firmware_replay
uv run --directory software/acoustic-trainer python tests/test_log_mel.py  # compiled ESP frontend for probe
python3 software/audio_ml/field_firmware_replay.py software/rover-monitor/logs/rover-monitor-20260925-004412.jsonl --output /tmp/field-consensus
software/acoustic-trainer/.venv/bin/python software/audio_ml/short_window_probe.py software/rover-monitor/logs/rover-monitor-20260925-004412.jsonl --manifest /tmp/field-labelled/wav/manifest.json --output /tmp/short-majority --majority-probe
```

## TARGETなしのモーター音収録（SW1デバッグ走行）

`firmware/ra8p1/SoundExplorationRover_CPU0/src/config/task_config.h` の
`CPU0_DEBUG_MOTOR_RECORDING_ENABLE=1U` でCPU0をビルドした場合だけ有効。今回の収録用ファーム設定は1。
SW1を**0.1秒以上・2秒未満押して離す**と、音を鳴らさなくても85 RPMで最大6秒間直進する。
走行中の同じ短押しで直ちに停止し、6秒経過しても自動停止。再走行には次の短押しが必要。
2秒以上の長押しは通常の現場学習操作（ただしデバッグ走行中は停止のみ）。

開始時も走行中も、通常の音源追従がLISTEN状態、TARGET一致保持なし、USB音響リンク稼働、学習・故障なし、
ToF/IMUとセンサー更新が正常であることを50 msごとに確認する。ToFは**正面550 mm・左右380 mm以上**を要求し、
値を下回ると停止して勝手に再始動しない。最終の車輪目標は従来通り `safety_arbiter_arbitrate()` を通す。
障害物を回り込む試験ではなく、距離不足で停止する低速直進録音。CPU1/駆動IPC/MRAM/音響判定基準は変更しない。

使い方:

1. Rover Monitorを先に起動する。ESP32のPCM診断が有効なら、ログに `record_type=acoustic_pcm` が保存される。
   既にPCMを録れているESP32は**そのまま**使用できる。ESP32も更新する場合は
   `APP_AUDIO_DATASET_STREAM_ENABLE=1U` のままで書き込み直す（0で再書き込みすると録音されない）。
2. CPU0のみ `bash .vscode/scripts/Invoke-RaBuild.sh --target CPU0` でビルドして書き込む
   （接続済みJ-Linkなら `--flash` を追加）。開けた場所で無音のままSW1を短押しする。
3. ログで `command.left_rpm` / `command.right_rpm` が85付近、実車輪RPMが非ゼロ、
   `recognition.status_name` がTARGETでない区間だけmotor-onlyとして採用する。
   新しいESP32と組み合わせれば `debug_motor.active=1` も記録される。
   旧ESP32でも既存のRPM/PCMログから同じ区間を抽出できる。
4. 収録後はCPU0のスイッチを0に戻して再ビルド・書込みする。短押しが意図せず自律走行を起こさない通常動作へ戻る。

無音でも走り出さない場合は `debug_motor.active`、USB/故障・センサー状態と正面/左右ToF距離を確認する。
音を鳴らさないままSW1を短押しした前後のログ時刻だけ共有すれば、motor-only PCMの切出しと評価はスクリプト側で行える。
