# I2S第2スロット: 4区間実機ログの結果（2026-09-25）

提供された `silence.jsonl` / `target.jsonl` / `target_noise.jsonl` / `motor.jsonl` をsample番号で照合し、欠損を跨がない2ch WAVを作成した。対になったPCMは約19.6/18.9/29.1/14.5秒。再現は `stereo_pcm_audit.py`（`--wav-dir`）→`stereo_spatial_probe.py`→`eval_stereo_query.py` / `eval_stereo_pretrained.py` の順。数値要約は [`stereo_field_results_20260925.json`](stereo_field_results_20260925.json)。録音本体や学習済みモデルはgitへ追加しない。

## 分かったこと

- 第2スロットは無音でもsampleの複製でもない。ただしアクティブな256-sampleブロックの左右絶対相関中央値は静音0.982、TARGET0.957、TARGET+雑音0.939、motor0.943。**独立した4マイクのraw PCMとみなす根拠はない**。左＋右のmidと差分sideでも、TARGET対motorの相対レベルはsideのほうが約0.83 dB悪い。
- 同一TARGETだけを5回登録し、両スロットに同じ照合手順（200 ms窓、80 ms更新、2-of-3投票）を適用した実測セッションでは、旧handcrafted照合のTARGET+雑音区間の受理が第1スロット180/349窓に対し第2スロット235/349窓。いずれも静音・motorのみ約28.8秒で受理0。ただしこれをRecall/FP/hourの証明とはしない。
- ESC-50の別音種で閾値を選んだ凍結query-conditioned小型CNN（14,249パラメータ、TARGET音種でPC学習せず）は、同じ録音で第1スロット10/342窓→第2スロット197/342窓を受理。TARGETのみでも4/207→155/207窓。静音/motorのみは両方0。ただし**未知カテゴリの独立ESC-50試験で約51 FP/hour**のままであり、音種を変えても低誤検知とは言えない。

`target_noise`全体はTARGETが間欠的に鳴った区間で、全窓にTARGETが存在するわけではない。さらに今回の4ログには**TVだけの第2スロット録音がない**。第2スロットの高い受理数はこの録音の改善傾向であり、TVだけをTARGETと誤認していないことや、未知TARGETへ一般化することを示さない。この段階でCPU0の走行判定を切り替えない。

## 人間の作業

**今すぐの追加収録・書込みは不要。** TVのみの第2スロットがないのは事実だが、現行CNNは別音種の独立試験で約51 FP/hourと不合格。今TVを20秒追加しても、未知TARGETで低誤検知というGO判断にはならない。先にAI側で一般音源の誤検知を減らすモデルを比較し、独立評価を通った場合だけ、現場受入れの最後の負例として「TVのみ20秒」の追加収録を依頼する。その時も収録ファームは既に書込済みで再フラッシュは不要。
