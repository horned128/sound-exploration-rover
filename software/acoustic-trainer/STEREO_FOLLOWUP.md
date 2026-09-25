# 4区間の2ch PCMと50音種事前学習の追試

## 実施結果

`silence`, `target`, `target_noise`, `motor` を2ch/16kHz PCMとして復元。対になった実音声は19.6/18.9/29.1/14.5秒。`stereo_pcm_audit.py`は欠損を跨がずWAVへ分割する。2スロットは完全に同一ではないが、256-sampleブロックの左右相関中央値0.939～0.982で、XVF3800の**未処理4マイク信号だとは示されていない**。mid/side差分はTARGET対motorのレベル差をmidより0.83 dB悪化させた。

今回の同一TARGETの5例を登録した200ms短窓では、`target_noise`でch0が180/349窓、ch1は235/349窓受理。別クラスで閾値を決めた14,249重みの小型query NNでも、ch0が10/342窓に対しch1が197/342窓（旧18音種事前学習版）。すなわち第2スロットには**このTARGETとこの雑音**を区別しやすくする情報がある。しかし、静音とmotorのみ約28.8秒で誤受理0という短い試験だけでは低FP/hourを立証できず、TVのみの2ch negativeもない。

一般化を改善するため、ESC-50を**44学習音種＋3選定音種＋3未知テスト音種**へ拡大（各12録音、計600 WAV）。いずれも現場TARGETは学習に含まれない。物理PCMでmixture augmentationして同じquery NNを再学習し、検証3音種でのみ閾値0.9/2-of-3投票を選んだ。別3音種での誤検知は51.43→**3.81 FP/hour**に改善したが、0dBで音源が確実に存在する混合例は**13/63**しか検出できない（選定側10/54）。同じ凍結モデルを今回の2ch現場録音に適用した結果は、`target_noise`でch0 **8/342**、ch1 **94/342**窓。静音/motorのみでは両方0。400msの音響窓＋80ms×2の投票だけで最低560ms、その後USB/推論時間が加わる。

**結論**: 第2スロットは現場の1音を改善するが、今回のモデルを使った「音種が事前に未知でも、同時発音を高感度・低誤検知・低遅延で判定する」GO条件は満たしていない。ch1を固定的な正解入力と決めることも、CPU0走行判定へ未合格CNNを組み込むこともしない。現状では追加録音よりも、複数マイクの未処理経路の有無と、より長時間・多種の一般音による事前学習/小型化の問題が残る。結果の機械可読版は [`stereo_followup_20260925.json`](stereo_followup_20260925.json)。

```sh
PY=software/acoustic-trainer/.venv/bin/python
TMP=/var/folders/__/d9pz1vws43n9zs95zsqwyf540000gp/T/opencode
$PY software/acoustic-trainer/fetch_general_audio.py --output "$TMP/esc50-all" --all-remaining-train
$PY software/acoustic-trainer/train_query_detector.py --manifest "$TMP/esc50-all/manifest.json" --output "$TMP/query-abs-all" --pairs 15000 --epochs 20 --frontend absolute --channels 24
$PY software/acoustic-trainer/eval_query_stream.py --esc "$TMP/esc50-all/manifest.json" --motor /tmp/opencode/labeled-two/wav/manifest.json --tv /tmp/opencode/field-labelled/wav/manifest.json --target /tmp/opencode/labeled-two/wav/manifest.json --model "$TMP/query-abs-all/query_abs_24.keras" --output "$TMP/query-abs-all-stream" --frontend absolute
$PY software/acoustic-trainer/eval_stereo_pretrained.py --silence "$TMP/silence-stereo" --target "$TMP/target-stereo" --target-noise "$TMP/target-noise-stereo" --motor "$TMP/motor-stereo" --model "$TMP/query-abs-all/query_abs_24.keras" --threshold 0.9 --output "$TMP/stereo-pretrained-all.json"
```
