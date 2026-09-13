# ハードウェア測定記録

この文書には、実機の寸法、校正値、駆動試験と、そこから採用した物理パラメータをまとめる。部品の仕様・購入情報は、引き続き対象コンポーネント配下の`hardware/*/spec/`を正とする。

## CPU1 駆動計測用一時ビルド

このビルドは、0-E「最小走行デューティ・実速度・惰走距離」を実機で測るための一時機能を有効にしたものです。
計測終了後は `DRIVE_MEASUREMENT_TEST_ENABLE` を `0U` に戻し、再度 Generate + Clean Build して通常版を書き戻します。

## 追加されたLive Watch変数

CPU1のデバッグコンテキストで、既存の同名登録を削除してから変数名で登録します。

| 変数 | 用途 |
|---|---|
| `g_drive_measurement_mode` | 0:通常、1:デューティ計測、2:強制停止 |
| `g_drive_measurement_duty_permille` | 計測入力デューティ。0〜560、単位は0.1% |
| `g_drive_measurement_applied_duty_permille` | 上限適用後の入力値 |
| `g_drive_measurement_status` | 0:無効、1:有効、2:強制停止、3:有効指令待ち、4:モード不正、5:ドライバ異常 |
| `g_drive_measurement_output_authorized` | 有効な通常指令を受信済みで計測出力を許可できるとき1 |
| `g_drive_measurement_stop_count_left` | 強制停止を適用した直前の左累積カウント |
| `g_drive_measurement_stop_count_right` | 強制停止を適用した直前の右累積カウント |
| `g_drive_measurement_stop_capture_valid` | 強制停止直前カウントが取得済みなら1 |
| `g_drive_left_duty_permille` | 左の現在デューティ。前進は負 |
| `g_drive_right_duty_permille` | 右の現在デューティ。前進は正 |
| `g_encoder_left_count` / `g_encoder_right_count` | 累積エンコーダカウント。前進は正 |
| `g_encoder_left_rpm_x10` / `g_encoder_right_rpm_x10` | 10倍RPM。前進は正 |
| `g_encoder_sample_elapsed_ms` | 直近RPM計算窓。通常100 ms |
| `g_encoder_sample_count` | RPM窓更新回数 |
| `g_task_actuator_elapsed_total_ms` | CPU1アクチュエータ実経過時間 |
| `g_actuator_service_fault_flags` | timeout等の安全状態 |

`g_drive_left_duty_permille` と `g_drive_right_duty_permille` は読み取り専用です。Live Watchから直接書き換えません。
書き換えるのは `g_drive_measurement_mode` と `g_drive_measurement_duty_permille` だけです。

## e² studioでの準備

1. まず車輪を浮かせ、モーター用外部電源とサーボ用外部電源をOFFにした状態で書き込みます。
   基板電源とJ10デバッグ接続は維持します。
2. CPU0/CPU1のデバッグを終了し、両プロジェクトをRefreshします。
3. CPU1の `src/config/drive_config.h` にある
   `DRIVE_MEASUREMENT_TEST_ENABLE (1U)` を確認します。
4. CPU0/CPU1でGenerate Project Contentを実行し、両方をClean Buildします。
5. `SoundExplorationRover Debug_Multicore Launch Group`でCPU0/CPU1の新しいELFを書き込み、両CPUをResumeします。
   CPU1 AttachだけではCPU1のフラッシュは更新されません。
6. 書き込み後、CPU1のLive Watchへ上表の変数を名前で登録します。古いアドレスの登録は使いません。

## 共通の開始条件

1. 最初は `g_drive_measurement_mode = 0` のまま、通常の前進音／通常指令を開始します。
   CPU0から有効な `actuator_enable` 指令が継続して届く状態にします。
2. 車輪を浮かせたまま、`g_drive_measurement_mode = 1`、`g_drive_measurement_duty_permille = 0` にします。
3. `g_drive_measurement_status = 1`、`g_drive_measurement_output_authorized = 1` を確認します。
   `status = 3` のままなら、通常の有効指令が届いていません。走行試験へ進みません。
4. 前進時の符号が `g_drive_left_duty_permille < 0`、`g_drive_right_duty_permille > 0` になることを確認します。
5. 実際に床で走らせる試験へ移るときだけ、周囲を片付け、非常停止できる位置でモーター電源をONにします。

`g_drive_measurement_duty_permille` は0〜560の範囲で使います。560を超えて書くと、
`g_drive_measurement_applied_duty_permille` は560になります。既存の左右倍率・上限のクランプを踏み越えない一時上限です。

## 1. 最小走行デューティ

車輪を床に置き、直進できる空間を確保します。最初から細かい機械的移動をする必要はありません。
Live Watchのデューティ値を変え、累積カウントの差分から動き出しを判定します。

1. `mode = 1`、デューティ0で、左右カウントと `g_task_actuator_elapsed_total_ms` を記録します。
2. デューティを `40, 80, 120, ... , 560` と順に設定します。
   各値を設定した後、最低1秒待ち、ランプが落ち着いてからさらに1〜2秒のカウント差分を記録します。
3. 各点で次を記録します: 入力値、applied値、左右現在デューティ、開始／終了カウント、経過時間、左右rpm_x10。
4. `abs(終了カウント - 開始カウント) > 0` となった最初の粗い区間を見つけます。
   その前後だけを `5〜10` 刻みで再測定し、左右それぞれの動き出し値を求めます。
5. 低速の1パルスだけをノイズと区別しにくい場合は、各点の観測時間を3秒へ延ばし、複数カウントの発生を動き出しとします。

左右で値が異なる場合は、左右別の最小走行デューティとして記録します。ここではまだ `DRIVE_DUTY_MIN_PERMILLE` を変更しません。

## 2. デューティと実速度の実測

車輪を床に置き、同じ直進条件で各デューティを比較します。`rpm_x10` は100 ms窓の値なので、
値が安定した区間の複数サンプルを使います。

1. デューティを `100, 200, 320, 400, 480, 560` と設定します。
2. 各点でランプが終わるまで約1秒待ち、その後5〜10秒走行します。
3. 各点について、開始／終了カウントと `g_task_actuator_elapsed_total_ms`、安定区間のrpm_x10を記録します。
4. カウントから再計算する場合は、次式を使います。

   `rpm_x10 = delta_count × 600000 / (702 × elapsed_ms)`

   `distance_mm = abs(delta_count) × 339.292 / 702`

5. 320‰の結果は、以前の120 RPM指令で得た約100〜105 RPMと比較します。指令RPMではなく、実測カウント／RPMを記録します。
6. 環境による速度変動を考慮し、左右の速度補正は行いません。左右は同じ速度として扱います。

   左右速度補正値は算出・ソース反映を行いません。環境による変動を考慮し、左右は同じ速度として扱います。

## 3. 320‰からの惰走距離

この試験では、モーター出力を切った後もエンコーダを読み続けます。

1. `mode = 1`、デューティ320で走行し、左右のカウントが安定して増えていることを確認します。
2. 速度が安定した時点の左右累積カウントを開始値として記録します。
3. `g_drive_measurement_mode = 1` のまま `g_drive_measurement_duty_permille = 0` にするのではなく、
   **`g_drive_measurement_mode = 2` へ変更します**。これがPWMを0にして惰走を開始する操作です。
4. `g_drive_measurement_stop_capture_valid = 1` を確認し、
   `g_drive_measurement_stop_count_left/right` を停止指令直前カウントとして記録します。
5. `g_drive_left_duty_permille` と `g_drive_right_duty_permille` が0になった後も、
   左右累積カウントを約1秒監視します。カウントが変化しなくなった値を最終値とします。
6. 惰走距離を次式で計算します。短い試験なのでカウンタ折返しは考慮不要です。

   `coast_count_left = abs(final_left - stop_count_left)`

   `coast_count_right = abs(final_right - stop_count_right)`

   `coast_distance_mm = coast_count × 339.292 / 702`

7. 左右それぞれの距離を記録し、S4の停止距離見通しには大きい方を使います。

惰走試験が終わったら、モーター電源をOFFにして通常指令を停止します。次の試験へ進むときは、
通常モードへ戻し、`mode = 0`、`duty = 0` を確認してから、必要なら基板を再起動して開始条件を作り直します。

## 異常時の扱い

- `status = 3` または `output_authorized = 0`: 通常指令未受信、timeout、または異常です。モーター電源をOFFにします。
- `g_actuator_service_fault_flags` のtimeout bitが立った場合、計測出力も停止します。CPU0の通常指令を復帰させてから再開します。
- `status = 4`: modeへ0〜2以外を書いています。直ちにmode2へ戻し、電源をOFFにします。
- `status = 5` またはdriver fault: 計測を中止し、電源をOFFにします。
- 予期せず動いた場合は、Live Watch操作を続けず、まずモーター用外部電源をOFFにします。

## 計測後の通常版への戻し方

1. モーター用外部電源とサーボ用外部電源をOFFにします。
2. デバッグを終了します。
3. `drive_config.h` の `DRIVE_MEASUREMENT_TEST_ENABLE` を `0U` に戻します。
4. Generate Project Content → Clean BuildをCPU1へ実施します。
5. CPU0/CPU1のMulticore Launch Groupで通常版ELFを書き込み、両CPUを起動します。
6. 新しい計測用Live Watch変数が表示されなくなり、通常の7変数だけが使えることを確認します。

## CPU1 エンコーダ校正 — 2026-09-12

### 測定条件

- ユーザーが車体を2000 mm手で押した。
- 各回の後、車体を持ち上げてスタート位置へ戻した。
- CPU1は動作状態。モーター駆動は行わず、累積カウントの増分を読む。
- 開始時の累積値は左 `1220`、右 `2450`。
- 車輪外径はSTLから確定した108.00 mm、外周は339.29 mmを使用。

### 生データ

| 回 | 左終了 | 右終了 | 左増分 | 右増分 |
|---:|---:|---:|---:|---:|
| 1 | 5428 | 6616 | 4208 | 4166 |
| 2 | 9573 | 10616 | 4145 | 4000 |
| 3 | 13671 | 14729 | 4098 | 4113 |
| 4 | 17787 | 18851 | 4116 | 4122 |
| 5 | 21877 | 22964 | 4090 | 4113 |
| 6 | 26022 | 27134 | 4145 | 4170 |
| 7 | 30198 | 31315 | 4176 | 4181 |
| 8 | 34320 | 35454 | 4122 | 4139 |
| 9 | 38508 | 39633 | 4188 | 4179 |
| 10 | 42646 | 43767 | 4138 | 4134 |

### 計算

```text
counts_per_rev = |count_delta| × 339.29 / 2000
```

| 側 | 平均増分/2000 mm | 標準偏差 | 変動係数 | 暫定 counts/rev | 暫定 mm/count |
|---|---:|---:|---:|---:|---:|
| 左 | 4142.6 | 38.5 | 0.93% | 702.77 | 0.482789 |
| 右 | 4131.7 | 53.2 | 1.29% | 700.92 | 0.484062 |
| 共通推定 | 4137.1 | — | — | **701.85** | — |

左右差は平均10.9カウント、0.26%。測定の再現性は良好で、校正前設定900との差は約22%ある。
この結果から、左右共通の実機校正値として **702 counts/rev** を採用する。

### 追加のRPM照合（実施不要）

一定速度でのストップウォッチ、左右累積カウント差、
`g_encoder_left_rpm_x10` / `g_encoder_right_rpm_x10`の同期測定は追加しない。
手押し10区間の再現性と現在までの実測値で確認完了とする。

### 追加測定: 10秒音源追従走行

ユーザーが追従移動時間を一時的に10秒へ変更し、前方から音を出して前進させた。
提示されたデータは開始値と4つの終了値で、見えている10秒区間は4回分である。

| 区間 | 左カウント増分 | 右カウント増分 |
|---:|---:|---:|
| 1 | 11849 | 12268 |
| 2 | 11931 | 12335 |
| 3 | 11664 | 12115 |
| 4 | 11800 | 12239 |
| 平均 | **11811.0** | **12239.25** |

各区間が実際に10秒間の走行であったと仮定すると、前回の暫定counts/revからの実RPMは
左約101.0、右約104.7となる。これは目標120 RPMに対する実速度であり、
速度フィードバック無効の現状では目標値と一致する必要はない。

走行終了後に読み取られた`g_encoder_left_rpm_x10` / `g_encoder_right_rpm_x10`は左右とも0だった。
これは停止後の最後の速度窓が0になった値であり、走行中の報告RPMとの比較には使えない。
`g_encoder_sample_count`の差分も区間ごとに一定でないため、今回の取得間隔をそのまま走行時間とは扱わない。

走行中に取得した`g_encoder_*_rpm_x10`は概ね780〜800だった。
これは校正前900設定での表示値であり、約78〜80 RPMに相当する。
停止後に読み取った値は0になるため、停止後の0は走行中のRPM比較には使わない。

厳密なストップウォッチ同期ログは追加取得しない。10秒走行のカウント測定と手押し測定から得た
701.85を丸め、ユーザー判断により`WHEEL_ENCODER_COUNTS_PER_REV=702`へ変更した。
変更後は同じカウント速度であれば、`g_encoder_*_rpm_x10`は約1000前後（約100 RPM）を示す見込みである。

### 採用結果

- `firmware/ra8p1/SoundExplorationRover_CPU1/src/config/drive_config.h`
  の`WHEEL_ENCODER_COUNTS_PER_REV`を`900U`から`702U`へ変更した。
- 変更理由は、2000 mm手押し10区間の共通推定701.85 counts/revと、左右差0.26%である。
- RPM絶対値の厳密な再照合は確認項目にせず、以後の計算・ドキュメントは702を基準とする。

## 車体走行幾何

最終更新: 2026-09-13

実機での旋回円の走行測定は行わず、左右独立駆動の差動二輪モデルで計算する。

### 採用値

| 量 | 値 |
|---|---:|
| 車輪外径 | 108 mm |
| 車輪半径 | 54 mm |
| 実効トレッド幅 `T` | 260 mm |
| 車体中心から左右車輪中心まで | 130 mm |

### 旋回半径

左右車輪の接地点速度を `vL`、`vR` とすると、車体中心の旋回半径 `R` は次式で計算する。

```text
R = (T / 2) × (vR + vL) / (vR - vL)
```

ここで `vR = vL` のときは直進なので、`R` は無限大として扱う。
`vR` と `vL` が同じ大きさで逆符号のときはその場旋回で、車体中心の `R` は0、
各車輪中心の旋回半径は `T / 2 = 130 mm` となる。

片側を停止して他側だけを動かす場合は、車体中心の旋回半径は `T / 2 = 130 mm`、
動いている側の車輪中心の半径は `T = 260 mm` である。

左右速度は環境による変動を含むため、制御上は補正せず同じ速度として扱う。
この寸法は表示用オドメトリの計算にも使用し、`software/rover-monitor/index.html`へ反映済み。
