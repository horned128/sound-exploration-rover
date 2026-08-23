# EK-RA8P1 アクチュエータ制御

CPU0から円軌道の制御指令を出し、CPU1がPWM出力とエンコーダ処理を担当します。

```text
CPU0 / Cortex-M85 / μT-Kernel
  usermain() ──> tk_init
                  ├─ tk_command (priority 6 / 50 ms)
                  │    └─ 4サーボ＋左右モーターを1フレームでIPC送信
                  └─ tk_think   (priority 10 / 100 ms)
                       ├─ 円軌道の目標生成
                       └─ 青・緑LEDによる状態／fault表示
                                      │
                                      v 意味ベースの32 bitメッセージ列
                                 IPC message FIFO channel 0
                                      │
                                      v
CPU1 / Cortex-M33 / bare metal
  IPC callback ──> actuator_app_run_1ms()
                    └─ drivers/{servo,dc_motor,encoder}
                         └─ FSP生成インスタンス / BSP API
```

FSPのIPC FIFOは4段、1要素32 bitです。上位8 bitをメッセージID、下位24 bitをペイロードとして使います。CPU0側は送信専用なのでIPC割り込みを使わず、CPU1側だけIPC受信割り込みとコールバックを有効にしています。CPU間の相互関係、起動順序、IPC、安全状態、FSP生成コードとの境界は[RA8P1ソフトウェア設計書](../../docs/ra8p1/ARCHITECTURE.md)を参照してください。

## 現在の動作

CPU1は4輪操舵サーボ、左右BTS7960、左右代表エンコーダを起動時に初期化します。モーターPWMは0～700 permilleへ制限します。

```c
#define MOTOR_PWM_MIN_DUTY_PERMILLE (0)
#define MOTOR_PWM_MAX_DUTY_PERMILLE (700)
```

購入時の仕様表記は100 rpmですが、今回の実機確認では約300 rpmを測定したため、開ループ換算上限を300 rpmへ変更しています。この値が無負荷測定である場合、車体搭載時の速度・トルクは別途確認してください。

CPU0はCPU1を起動し、ワンショットの`tk_init`を`tk_cre_tsk()`で生成します。`tk_init`は独立した`tk_think`と`tk_command`をそれぞれ`tk_cre_tsk()`で生成・開始してから自己削除します。`tk_think`は直進保持、円軌道操舵、円走行の順に目標を生成します。モーター指令のRPMは、閉ループ速度制御ではなくオープンループPWM換算値です。

`tk_command`は高優先度の50 ms周期で最新目標を読み、FR、FL、RR、RLと左右モーターの6指示値を1つのIPCスナップショットとして送信します。思考目標が500 ms途絶した場合は緊急停止へ切り替えます。

CPU1は受信したFR、FL、RR、RLの角度をそれぞれ1200～1800 usへ変換します。FSPには`g_servo_pwm_fr`、`g_servo_pwm_fl`、`g_servo_pwm_rr`、`g_servo_pwm_rl`を登録済みです。共有ピン設定はSolutionで一元管理し、CPU0の`g_bsp_pin_cfg`が起動時に適用します。そのためCPU1の`pin_data.c`が0ピンなのは正常です。起動直後はPWMを開始せず、有効な指令を受けてから出力します。CPU1はCPU0の青・緑LEDと競合しないよう、赤LEDだけをheartbeatに使用します。

### 左右論理の反転

実機を前方から見た左右と、初期ソフトの左右定義が反対だったため、CPU0/IPCで使う論理FR/FL/RR/RLおよび左右モーター・エンコーダの対応を入れ替えています。FSP生成インスタンス名も論理名に合わせて整理済みです（`g_servo_pwm_fr`は論理FR、`g_encoder_left_a_irq`は論理左A相）。ピン番号とSolutionの共有ピン設定は変更していません。

## EK-RA8P1のスイッチ設定

電源を切ってからSW4を次のように設定します。

| スイッチ | 設定 | 理由 |
|---|---|---|
| SW4-3 | ON | Octo-SPIを無効化 |
| SW4-4 | ON | Arduino/mikroBUS端子を有効化 |

EK-RA8P1は既定でSW4-4がOFFで、Arduinoヘッダが切り離されています。Arduino端子とOcto-SPIは同時使用できません。詳細は[EK-RA8P1 v1 User's Manual](https://www.renesas.com/en/document/mat/ek-ra8p1-v1-users-manual)のSW4設定とArduino Connectorの章を参照してください。

この配線ではP801（OSPI DQS）、P803（OSPI SIO1）、P808（OSPI SCLK）をサーボPWMへ転用するため、Octo-SPIフラッシュは使用できません。Octo-SPIを再び使う場合は、FL/RR/RLの3信号を別のGTIOC候補へ移してからSW4-3をOFFへ戻してください。

## DS3225MGの配線

| サーボ | FSPインスタンス | 信号接続 | GPT出力 |
|---|---|---|---|
| 論理FR | `g_servo_pwm_fr` | J26-4、Pmod1 SCK、P803 | GPT12B |
| 論理FL | `g_servo_pwm_fl` | J24-2、Arduino D9、P110 | GPT9B |
| 論理RR | `g_servo_pwm_rr` | J26-2、Pmod1 MOSI、P801 | GPT11B |
| 論理RL | `g_servo_pwm_rl` | J23-1、Arduino D0、P808 | GPT13B |

`SoundExplorationRover/solution.xml`の上記4ピンを正とします。CPU0/CPU1のPins画面で同じピンを重複設定せず、Generate Project Content後はCPU0の`ra_gen/pin_data.c`でP110/P803/P808/P801が`IOPORT_PERIPHERAL_GPT1`、CPU1の`ra_gen/pin_data.c`が0ピンであることを確認します。

### CPU0/CPU1の書き込み

ピン設定を変更した後はCPU0、CPU1の順にGenerate Project Contentを実行し、両プロジェクトをCleanしてから両方をBuildします。書き込みには`SoundExplorationRover Debug_Multicore Launch Group`を使います。この構成はCPU0の`Debug/SoundExplorationRover_CPU0.elf`とCPU1の`Debug/SoundExplorationRover_CPU1.elf`を組で読み込むため、別フォルダーに残った古いSRECを個別選択しないでください。

サーボの赤線は外部安定化電源+4.8～6.8 V、黒線は外部電源GNDへ接続し、外部電源GNDとEK-RA8P1のGNDを共通化します。

```text
EK-RA8P1 J24-2 (D9/P110) ---------------- 白 PWM

外部 5～6 V / 3 A以上  + ---------------- 赤
外部 5～6 V / 3 A以上 GND --------------- 黒
                         |
EK-RA8P1 J18-6/7 GND ----+
```

サーボの赤線をEK-RA8P1の5 V端子から給電しないでください。DS3225MGはストール付近で約3 Aに達し得るため、外部電源を使い、基板とサーボのGNDだけを共通化します。初回はホーンやリンクを外して動作範囲を確認してください。DS3225系の資料値は500～2500 us、中央1500 us、50～330 Hzですが、このサンプルは安全側の1000～2000 us、50 Hzから開始します。[メーカーのDS3225仕様書](https://www.dsservo.com/show_down.asp?id=24)

### 4輪の原点補正

1. 車体を浮かせ、リンクまたはホーンを外して0度指令を出す。
2. 各ホーンを直進に最も近い歯へ付け直す。
3. CPU1をデバッグし、`g_servo_center_trim_us[0..3]`を5～10 usずつ変更して直進へ合わせる。
4. 決まった値を`SERVO_CENTER_TRIM_US_FR/FL/RR/RL`へ転記する。

操舵方向が1輪だけ逆なら、対応する`SERVO_DIRECTION_FR/FL/RR/RL`を`-1`へ変更します。

## JGA25-370の配線

### エンコーダ

現在の6輪構成は、論理左3台を論理左BTS7960、論理右3台を論理右BTS7960へ並列接続するため、PWMを6台個別には制御していません。この段階では各側の代表モーターを1台ずつ測定します。6台分のA/Bを1本へまとめると、位相・速度の異なる信号が衝突するため禁止です。

| 測定対象 | JGA25-370信号 | EK-RA8P1 | 用途 |
|---|---|---|---|
| 論理左代表 | 黄（A相） | J23-3、Arduino D2、P011/IRQ16 | 両エッジ割り込み |
| 論理左代表 | 緑（B相） | Arduino D1、J23-2、P809/IRQ20 | 両エッジ割り込み |
| 論理右代表 | 黄（A相） | Pmod1 J26-7、P006/IRQ11 | 両エッジ割り込み |
| 論理右代表 | 緑（B相） | Pmod1 J26-10、P413/IRQ18 | 両エッジ割り込み |
| 各エンコーダ | 青（エンコーダVCC） | J18-4、3.3 V | エンコーダ電源（各モーターへ分配） |
| 各エンコーダ | 黒（エンコーダGND） | J18-6またはJ18-7、GND | EK-RA8P1と共通GND |

A相だけでも指令方向を前提に速度の大きさは測れますが、逆転判定と4逓倍カウントにはA/B両方が必要です。本ソフトは左右ともA/Bの両相を両エッジ割り込みで読み取ります。未配線の入力を浮かせると誤カウントするため、エンコーダを接続しない状態で走行させないでください。

購入品は300 RPM品であり、旧100 RPM品を仮定した2024 count/revは使用できません。出力軸2回転で約1800カウントを実測したため、`JGA25_ENCODER_COUNTS_PER_REV=900`として100 msごとに左右のRPMを算出します。より正確な値が必要な場合は、出力軸10回転の平均で再校正します。測定方法は[JGA25-370 12 V・300 RPM仕様書](../../hardware/actuator/docs/jga25-370-12v-300rpm.md)を参照してください。

`g_jga25_left_encoder_count`と`g_jga25_right_encoder_count`は、左右とも前進で増加し、後進で減少する4逓倍累積カウントです。`g_jga25_left_rpm_x10`と`g_jga25_right_rpm_x10`も同じ符号規則の推定RPMの10倍であり、`-1234`は後進方向の`-123.4 RPM`を表します。実測RPMは状態取得に使用し、PWM制御はオープンループです。

最終的に6台すべての速度を個別に閉ループ制御する場合は、6組（A/B計12入力）と6チャンネルのモーター駆動、または各側のエンコーダ値を集約する外部回路が必要です。現行の左右2台のBTS7960構成では、まず左右代表2組で十分です。

### モータードライバ

DCモーターをEK-RA8P1へ直接接続してはいけません。左右それぞれ1台のBTS7960モジュールを使い、同じ側のモーターを各ドライバのM+/M-へ接続します。

| EK-RA8P1 | 論理左BTS7960 | 論理右BTS7960 | 用途 |
|---|---|---|---|
| J23-5、D4、P810/GTIOC10A | — | RPWM | 論理右20 kHz後退PWM |
| J23-4、D3、P811/GTIOC10B | RPWM | — | 論理左20 kHz後退PWM |
| Pmod2 J25-3、P602/GTIOC7B | LPWM | — | 論理左20 kHz前進PWM |
| Pmod2 J25-2、P603/GTIOC7A | — | LPWM | 論理右20 kHz前進PWM |
| J24-1、D8、PD01 | R_ENとL_EN | R_ENとL_EN | 左右共通EN |
| J23-8、D7、P312 | 未接続 | 未接続 | 解放（予備GPIO） |
| J18-5、+5 V | VCC | VCC | BTS7960モジュール論理電源を共通化 |
| J18-6/J18-7、GND | GND | GND | 信号・電源共通GND |
| 外部モーター電源 | B+ / B- | B+ / B- | モーター電源 |
| 各側のDCモーター | M+ / M- | M+ / M- | ドライバ出力 |

左右BTS7960のVCCは共通のロジック電源へ接続し、各モジュールのR_ENとL_ENを左右ともPD01へ接続します。VCCとENを直結するのではなく、CPU1がPD01をHigh/Low制御する構成です。P312/D7は配線せず、Solutionと生成ピン設定でも未使用にします。PD01とBTS7960のEN入力を接続したまま、PD01をVCCへ直接接続しないでください。CPU1のsafe stop、IPCタイムアウト、起動時Low保持を有効にするためです。

電源はArduino電源コネクタJ18から分岐できますが、電圧を混在させないでください。BTS7960のVCCはモジュール仕様で5 Vが要求される場合が多いため、仕様が3.3 V対応と明記されていない限りJ18-5（+5 V）へ接続します。左右2台のBTS7960のVCCをそこから分岐し、左右代表エンコーダのVCCはJ18-4（+3.3 V）から分岐します。4機器のGNDはJ18-6またはJ18-7から分岐して共通化します。J18-4とJ18-5は接続せず、エンコーダ出力が3.3 Vを超えないことを確認してください。BTS7960のVCC電流がEK-RA8P1の電源容量を超える場合は、外部の安定化5 V電源を使い、GNDだけを共通化します。

RPWMとLPWMは同じ側で同時にHighにせず、同じ側では1方向のPWMだけを出します。現在の実機配線では、論理正RPM（前進）を左右ともLPWMへ変換します。負RPM（後退）は左右ともRPWMへ変換します。前後基準の設定は`MOTOR_CHASSIS_FORWARD_SIGN`、左右個別の補正は`MOTOR_LEFT_MOUNT_SIGN`、`MOTOR_RIGHT_MOUNT_SIGN`です。P602/P603はPmod2へ接続し、Octo-SPI（OSPI0）のP100～P106、P800～P804とは別のOSPI1系ピンを使用しています。BTS7960のVCC、EK-RA8P1、外部モーター電源は必ずGNDを共通化します。

左右RPM指令は実測回転数をフィードバックせず、PWMデューティへ換算します。そのため、同じ指令でもモーター個体差、ギア・ハブの抵抗、タイヤ荷重、BTS7960や配線の電圧降下によって実回転数は一致しません。特に20%付近の低デューティでは静止摩擦の影響が大きく、片側の速度差が目立ちます。

## 円走行の動作確認

1. 最初は車輪を浮かせ、モーター電源を電流制限付きにする。
2. CPU0、CPU1の両方を書き込んでリセットする。
3. 約0.3秒後までにサーボPWMが有効になり、直進位置を保持する。
4. 約1.5秒後に前輪-45度、後輪+45度へ操舵する。
5. 約3.0秒後に左右+100 RPM相当でモーターが同時にソフトスタートする。

目標値はRPMという名前ですが、現在は20 kHz PWMへのオープンループ換算値です。円軌道は`CPU0_CIRCLE_LEFT_RPM`、`CPU0_CIRCLE_RIGHT_RPM`、`CPU0_CIRCLE_STEERING_DEG`で調整します。現在はソフトウェア上の安全限界である±45度を使うため、リンク機構が干渉する場合は角度を小さくしてください。

### CPU0状態LED

`tk_think`が青LEDと緑LEDを所有し、赤LEDはCPU1が所有します。

| LED表示 | 意味 |
|---|---|
| 青: 500 msごとに反転 | 直進位置へ原点合わせ中 |
| 青: 100 msごとに反転 | 円軌道の操舵位置へ移動中 |
| 青: 点灯 | 円走行中 |
| 緑: 1秒ごとに100 ms点灯 | `tk_think`が正常に周期実行中 |
| 緑: 点灯＋青: 回数点滅 | CPU0 fault。青の点滅回数がfault番号 |
| 赤: 500 msごとに反転 | CPU1正常heartbeat |
| 赤: 約50 msごとに反転 | CPU1でFSP/driverエラー発生 |

CPU0 fault番号は、1回がタスク生成／開始、2回がIPC初期化、3回がIPC送信、4回が思考目標タイムアウト、5回が目標共有失敗です。

CPU1をデバッグして、次の変数をLive Watchへ追加すると確認しやすくなります。

| 変数 | 内容 |
|---|---|
| `g_servo_pulse_us[0..3]` | 各サーボの現在の指令幅 |
| `g_drive_left_duty_permille` | 左モーターの符号付きPWM指令（負値はLPWM） |
| `g_drive_right_duty_permille` | 右モーターの符号付きPWM指令（負値はLPWM） |
| `g_jga25_left_encoder_count` / `g_jga25_right_encoder_count` | 左右代表モーターの4逓倍累積カウント（前進正、後進負） |
| `g_jga25_left_rpm_x10` / `g_jga25_right_rpm_x10` | 左右代表モーター推定RPMの10倍（前進正、100 ms更新） |
| `g_actuator_last_error` | 最後のFSPエラー |
| `g_actuator_fault_flags` | タイムアウト、緊急停止、指令制限、IPC・ドライバー異常フラグ |

CPU0では次の変数をLive Watchへ追加します。

| 変数 | 内容 |
|---|---|
| `g_cpu0_think_state` | 原点合わせ、円軌道操舵、円走行、fault |
| `g_cpu0_think_cycle_count` | 思考タスクの実行回数 |
| `g_cpu0_fault_flags` | CPU0でラッチした異常ビット |
| `g_cpu0_command_sequence` | 最終IPCシーケンス番号 |
| `g_cpu0_command_send_count` | 正常にcommitした指令数 |
| `g_cpu0_command_last_error` | 最後のIPC FSPエラー |

回転方向やカウント符号が意図と逆なら、モーターのOUT1/OUT2、A/B相、またはソフトウェアの符号規約のいずれか一つだけを入れ替えます。

## 主な実装箇所

| ファイル | 役割 |
|---|---|
| `common/ipc_message.h` | 両CPU共通のコマンド型と32 bit通信形式 |
| `SoundExplorationRover_CPU0/src/app/main.c` | CPU1起動とCPU0タスク群の起動 |
| `SoundExplorationRover_CPU0/src/app/tasks/tk_init.c` | 独立タスクの資源生成、`tk_cre_tsk()`、開始、失敗時の後処理 |
| `SoundExplorationRover_CPU0/src/app/tasks/tk_think.c` | 円軌道の目標生成、青・緑LED、CPU0 faultラッチ |
| `SoundExplorationRover_CPU0/src/app/tasks/tk_command.c` | 最新目標の共有、期限監視、全アクチュエータのIPC一括送信 |
| `SoundExplorationRover_CPU0/src/ipc/` | CPU0側IPCコマンド送信 |
| `SoundExplorationRover_CPU1/src/app/` | CPU1初期化、周期処理、フェイルセーフ |
| `SoundExplorationRover_CPU1/src/drivers/` | DCモーター、エンコーダ、サーボとFSP/BSP APIの呼び出し |
| `SoundExplorationRover_CPU1/src/ipc/` | CPU1側IPCコマンド受信・コミット |
| `SoundExplorationRover_CPU0/src/cpu0_config.h` | CPU0周期、優先度、円軌道、LED設定 |
| `SoundExplorationRover_CPU1/src/cpu1_config.h` | ピン、周期、制限値、安全タイムアウト |
| `SoundExplorationRover_CPU1/src/hal_entry.c` | 生成コードとユーザーアプリを接続する入口 |

### CPU0タスク一覧

`tk_init`は起動時だけ動く独立カーネルタスクです。共有資源を作成し、`tk_command`と`tk_think`を個別に生成・開始した後、`tk_exd_tsk()`で自己削除します。3タスクはそれぞれ`tk_cre_tsk()`で生成され、別々のスタックと優先度を持ちます。

| ファイル | カーネルタスク | 優先度 | 周期・待ち | 役割 |
|---|---|---:|---|---|
| `tk_init.c` | `cpu0_init_task` | 5 | 起動時に1回 | 共有資源と子タスクを生成・開始後に自己削除 |
| `tk_command.c` | `cpu0_command_task` | 6 | 50 ms | CPU1へ6出力分の最新目標を送信 |
| `tk_think.c` | `cpu0_think_task` | 10 | 100 ms | 円軌道目標、LED、faultラッチ |

IPCの構成方法と、送信側は割り込み不要・受信側はコールバックが必要という条件は、Renesasの[Getting Started with IPC on Dual Core MCU](https://www.renesas.com/en/document/apn/getting-started-ipc-dual-core-mcu)に準拠しています。
