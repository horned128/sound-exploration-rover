# EK-RA8P1 アクチュエータ制御

CPU0がReSpeaker/XIAOからUSBで音響観測を受けて短距離の音源追従目標を生成し、CPU1がPWM出力とエンコーダ処理を担当します。この文書は主にアクチュエータ配線と単体確認を扱います。全体構造は[ローバー ファームウェア設計書](../../docs/firmware/ARCHITECTURE.md)、USB接続、音響protocol、DoA校正、音源追従試験は[ReSpeaker統合設計](../../docs/firmware/RESPEAKER_INTEGRATION.md)、I2Cセンサーとルールベース走行は[I2Cセンサー・ルールベース走行](../../docs/firmware/SENSOR_AUTONOMY.md)を参照してください。

2026-09-25の音響見本はI2S右slotの32-bin log-melを用いる。CPU0はESP HELLOの`ACOUSTIC_CAPABILITY_FEATURE_SLOT1`を必須にし、MRAM見本version 5の5例が保存されるまで、DoA/VADだけによる自律発進を認めない。旧version 4の左slot見本は再学習が必要。[同じCPU0 C照合での実録音比較](../../software/acoustic-trainer/BASELINE_SLOT1_DEPLOYMENT.md)に手順を記す。

```text
CPU0 / Cortex-M85 / μT-Kernel
  usermain() ──> CPU0 task registry
                   ├─ task_command (priority 6 / 50 ms)
                   │    └─ 4サーボ＋左右モーターを1フレームでIPC送信
                   ├─ task_acoustic_link   (priority 8 / 1 ms poll)
                   │    └─ J7 USB HCDCの音響観測受信・J11 AI Lab CDC deviceの診断
                   └─ task_think   (priority 10 / 100 ms)
                        ├─ 停止聴取型の音源追従目標
                        └─ 青・緑LEDによる状態／fault表示
                                      │
                                      v 意味ベースの32 bitメッセージ列
                                 IPC message FIFO channel 0
                                      │
                                      v
CPU1 / Cortex-M33 / μT-Kernel
  usermain() ──> CPU1 task registry
                   ├─ task_actuator (priority 4 / 周期通知1 ms、実Δt更新)
                   │    └─ actuator_service ──> services/drive_service ──> drivers/{bts7960,encoder}
                   └─ task_status   (priority 12 / 10 ms)
                        └─ 赤LED heartbeat／fault表示
```

FSPのIPC FIFOは方向ごとに4段、1要素32 bitです。上位8 bitをメッセージID、下位24 bitをペイロードとして使います。CPU0→CPU1は走行指令、CPU1→CPU0は実デューティ・エンコーダRPM・fault・適用済み指令sequenceの診断に使い、両CPUで受信割り込みとコールバックを有効にします。

## μT-Kernel設定の配置と上書き

`common/mtk3_bsp2/`はμT-Kernel BSPそのものです。ローバー固有の設定をBSPへ直接書き込むと、BSP更新時に差分が失われ、CPU0/CPU1の設定差も追いにくくなります。このため、本プロジェクトの設定はBSPの外に置き、コンパイラのinclude探索順で標準定義を上書きします。

```text
CPU0 project
  mtkernel_config  ──リンク──> ../common/mtkernel/
                                     └─ config.h             : CPU0/CPU1共通設定
  mtk3_bsp2        ──リンク──> ../common/mtk3_bsp2/          : μT-Kernel BSP標準

CPU1 project
  mtkernel_config  ──リンク──> ../common/mtkernel/
                                     └─ config.h             : CPU0/CPU1共通設定
  src/mtkernel_config/include/                                : CPU1固有の上書き
    └─ sys/sysdepend/ra_fsp/
         ├─ cpu/ra8p1/sysdef.h
         └─ ek_ra8p1/{machine.h,sysdef.h}
  mtk3_bsp2        ──リンク──> ../common/mtk3_bsp2/          : μT-Kernel BSP標準
```

e² studio上の`mtkernel_config`は物理フォルダではなく、両CPUの`.project`で`../common/mtkernel/`へ張ったリンク名です。従ってCPU0にも共通設定は存在しますが、`CPU0/src/mtkernel_config/`という実フォルダはありません。CPU1だけはさらに固有定義を必要とするため、実フォルダの`src/mtkernel_config/include/`を持ちます。

### include探索順と責務

FSP/BSPの設定を含む順序は、CPU0では`common/mtkernel`、`common/mtk3_bsp2/config`、`common/mtk3_bsp2/include`です。CPU1では`common/mtkernel`の後、BSPの前に`src/mtkernel_config/include`も追加されます。`config.h`は共通フォルダから読み込まれ、CPU1固有の同名`sys/...`ヘッダはBSP標準ヘッダより先に見つかります。つまり、共通設定はBSP標準を上書きし、CPU1固有設定は対象のCPU／ボード依存ヘッダだけを追加で上書きします。

| 配置 | 対象 | 設定する内容 | 必要な理由 |
|---|---|---|---|
| `common/mtkernel/config.h` | CPU0/CPU1共通 | システム領域、最大優先度、カーネルオブジェクト上限、デバイス管理上限、例外・一時スタック、メモリ管理、引数検査、FPU利用 | ローバーで使うμT-Kernelの資源量と検査方針をBSP更新から独立して管理する。 |
| 同 `CNF_TIMER_PERIOD` | CPU0/CPU1共通 | カーネル時刻の基本周期を1 ms | `tk_dly_tsk()`などのアプリケーション待ち時間をミリ秒単位で扱う。BSP標準の10 msより細かい周期が必要なため。 |
| 同 `_RA_ORDINAL`分岐 | CPU0/CPU1共通ファイル内のCPU別設定 | CPU0はT-Monitorとシステム／例外メッセージを有効化、CPU1は無効化 | 両コアがBSPの単一SCI8 T-Monitor端子を同時に使用しないようにする。`_RA_ORDINAL=1`がCPU0、`=2`がCPU1。 |
| `CPU1/src/mtkernel_config/include/sys/sysdepend/ra_fsp/ek_ra8p1/machine.h` | CPU1のみ | CPU1をCortex-M33（Armv8-M）として選択し、CPU／ターゲット依存ヘッダの探索先を指定 | BSPのCPU0向けCortex-M85定義をCPU1へ適用しないため。 |
| `CPU1/src/mtkernel_config/include/sys/sysdepend/ra_fsp/cpu/ra8p1/sysdef.h` | CPU1のみ | CPU1 SRAMを`0x220EA000`から`0x000EA000` byte（936 KiB）とし、初期スタック位置、割込み数・優先度、SysTick許容周期、FPU有無を定義 | FSPのマルチコアメモリ分割で、CPU1にはSRAM上位936 KiBだけが割り当てられるため。 |
| `CPU1/src/mtkernel_config/include/sys/sysdepend/ra_fsp/ek_ra8p1/sysdef.h` | CPU1のみ | CPUCLK／ICLKを250 MHz、各PCLKをFSP設定に合わせて定義し、`SYSCLK`とタイマクロックを導出 | CPU1のCortex-M33はPLL1P÷4で250 MHz動作のため。CPU0標準の1 GHz前提を使うとカーネル時刻が一致しない。 |

FSPの生成済みリンク設定では、CPU0のSRAMは`0x22000000`から936 KiB、CPU1のSRAMは`0x220EA000`から936 KiBです。CPU0はCortex-M85・1 GHz・SRAM下位領域というBSP標準の前提と一致するため、共通の`config.h`だけで動作します。一方、CPU1はコア、クロック、SRAM領域がいずれも異なるため、3個の固有ヘッダによる上書きが必要です。

この構成では、`common/mtk3_bsp2/`を編集・上書きしてはいけません。BSPを更新する場合は、更新後にCPU0/CPU1をClean Buildし、この節のinclude順とCPU1の3個の上書きヘッダが維持されていることを確認してください。

## 現在の動作

CPU1は4輪操舵サーボ、左右BTS7960、左右代表エンコーダを起動時に初期化します。モーターPWMは0～700 permilleへ制限します。

```c
#define MOTOR_PWM_MIN_DUTY_PERMILLE (0)
#define MOTOR_PWM_MAX_DUTY_PERMILLE (700)
```

開ループの速度換算上限は`DRIVE_TARGET_RPM_MAX=300` rpmです。

CPU0はCPU1を起動した後、μT-Kernel初期タスクから呼ばれる`usermain()`で`task_registry_init()`を実行します。`task_registry.c`の登録配列を基に`task_think`、`task_command`、`task_acoustic_link`の全リソースを生成してから全タスクを開始し、`usermain()`は永久休止します。CPU1も独立したμT-Kernelを起動し、CPU1側`usermain()`から`task_actuator`と`task_status`を生成・開始して永久休止します。`task_think`は停止中の音響観測が閾値と方向安定条件を満たした場合だけ、短時間の操舵・前進目標を生成します。CPU1は左右代表エンコーダの実測RPMを診断へ返します。速度フィードバックは無効で、実測RPMは診断にのみ使用します。

`task_command`は高優先度の50 ms周期で最新目標を読み、FR、FL、RR、RLと左右モーターの6指示値を1つのIPCスナップショットとして送信します。思考目標が500 ms途絶した場合は緊急停止へ切り替えます。

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

ピン設定を変更した後はCPU0/CPU1 projectをRefreshし、CPU0、CPU1の順にGenerate Project Contentを実行し、両プロジェクトをCleanしてから両方をBuildします。書き込みには`SoundExplorationRover Debug_Multicore Launch Group`を使います。この構成はCPU0の`Debug/SoundExplorationRover_CPU0.elf`とCPU1の`Debug/SoundExplorationRover_CPU1.elf`を組で読み込むため、別フォルダーに残った古いSRECを個別選択しないでください。

CPU1のμT-Kernel sourceを含めた現在のビルド手順は、Refresh、Generate Project Content、Clean、Buildの順です。CPU0/CPU1のELFは`SoundExplorationRover Debug_Multicore Launch Group`で組にして書き込みます。詳細は[ローバー ファームウェア設計書のビルド節](../../docs/firmware/ARCHITECTURE.md#11-ビルド生成書き込み)を参照してください。

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

`WHEEL_ENCODER_COUNTS_PER_REV=702`として、100 msごとに左右のRPMを算出します。この値は2000 mm手押し10区間から得た実機校正値です。モーターの公称仕様は[JGA25-370 12 V・300 RPM仕様書](../../hardware/actuator/spec/jga25-370-12v-300rpm.md)を参照してください。

`g_encoder_left_encoder_count`と`g_encoder_right_encoder_count`は、左右とも前進で増加し、後進で減少する4逓倍累積カウントです。`g_encoder_left_rpm_x10`と`g_encoder_right_rpm_x10`も同じ符号規則の推定RPMの10倍であり、`-1234`は後進方向の`-123.4 RPM`を表します。現在は`DRIVE_SPEED_FEEDBACK_ENABLE=0`として実測RPMを診断だけに使用します。

最終的に6台すべての速度を個別に閉ループ制御する場合は、6組（A/B計12入力）と6チャンネルのモーター駆動、または各側のエンコーダ値を集約する外部回路が必要です。現行の左右2台のBTS7960構成では、まず左右代表2組で十分です。

### モータードライバ

DCモーターをEK-RA8P1へ直接接続してはいけません。左右それぞれ1台のBTS7960モジュールを使い、同じ側のモーターを各ドライバのM+/M-へ接続します。

| EK-RA8P1 | 論理左BTS7960 | 論理右BTS7960 | 用途 |
|---|---|---|---|
| J23-5、D4、P810/GTIOC10A | — | RPWM | 論理右20 kHz前進PWM |
| J23-4、D3、P811/GTIOC10B | RPWM | — | 論理左20 kHz後退PWM |
| Pmod2 J25-3、P602/GTIOC7B | LPWM | — | 論理左20 kHz前進PWM |
| Pmod2 J25-2、P603/GTIOC7A | — | LPWM | 論理右20 kHz後退PWM |
| J24-1、D8、PD01 | R_ENとL_EN | R_ENとL_EN | 左右共通EN |
| J23-8、D7、P312 | 未接続 | 未接続 | 解放（予備GPIO） |
| J18-5、+5 V | VCC | VCC | BTS7960モジュール論理電源を共通化 |
| J18-6/J18-7、GND | GND | GND | 信号・電源共通GND |
| 外部モーター電源 | B+ / B- | B+ / B- | モーター電源 |
| 各側のDCモーター | M+ / M- | M+ / M- | ドライバ出力 |

左右BTS7960のVCCは共通のロジック電源へ接続し、各モジュールのR_ENとL_ENを左右ともPD01へ接続します。VCCとENを直結するのではなく、CPU1がPD01をHigh/Low制御する構成です。P312/D7は配線せず、Solutionと生成ピン設定でも未使用にします。PD01とBTS7960のEN入力を接続したまま、PD01をVCCへ直接接続しないでください。CPU1のsafe stop、IPCタイムアウト、起動時Low保持を有効にするためです。

電源はArduino電源コネクタJ18から分岐できますが、電圧を混在させないでください。BTS7960のVCCはモジュール仕様で5 Vが要求される場合が多いため、仕様が3.3 V対応と明記されていない限りJ18-5（+5 V）へ接続します。左右2台のBTS7960のVCCをそこから分岐し、左右代表エンコーダのVCCはJ18-4（+3.3 V）から分岐します。4機器のGNDはJ18-6またはJ18-7から分岐して共通化します。J18-4とJ18-5は接続せず、エンコーダ出力が3.3 Vを超えないことを確認してください。BTS7960のVCC電流がEK-RA8P1の電源容量を超える場合は、外部の安定化5 V電源を使い、GNDだけを共通化します。

RPWMとLPWMは同じ側で同時にHighにせず、同じ側では1方向のPWMだけを出します。論理正RPM（前進）は左LPWM（P602/GPT7B）と右RPWM（P810/GPT10A）、負RPM（後進）は左RPWM（P811/GPT10B）と右LPWM（P603/GPT7A）へ出します。前進時の物理回転は左が正回転（右回り）、右が逆回転（左回り）で、左右Encoderは前進正・後進負です。設定は`DRIVE_CHASSIS_FORWARD_SIGN=-1`、`DRIVE_LEFT_MOUNT_SIGN=+1`、`DRIVE_RIGHT_MOUNT_SIGN=-1`です。P602/P603はPmod2へ接続し、Octo-SPI（OSPI0）のP100～P106、P800～P804とは別のOSPI1系ピンを使用しています。BTS7960のVCC、EK-RA8P1、外部モーター電源は必ずGNDを共通化します。

左右RPM指令は基本PWMデューティへ換算し、`DRIVE_LEFT_DUTY_SCALE_PERMILLE=700`、`DRIVE_RIGHT_DUTY_SCALE_PERMILLE=250`を適用します。代表エンコーダ実測RPMによる比例補正機構は`DRIVE_SPEED_FEEDBACK_ENABLE=0`で無効化しています。左右各3台を1台のBTS7960へ並列接続しているため、補正対象は各側の代表モーターであり、3台個別の速度一致は保証しません。左右RPMがともに0の停止指令では、ランプダウンを待たずPWMと共通ENを即時に停止します。

## 音源追従の動作確認

初回は車輪を浮かせ、モーター電源を電流制限付きにします。USB linkと音響観測が成立するまでCPU0はemergency stopを維持します。成立後は停止して音を聴き、DoA更新待ちと5点の安定確認を行います。前方・側方ではservoを500 ms整定して最大左右120 RPM相当で前進します。後方ではハの字操舵を500 ms整定して左右逆転のその場旋回を行い、車体中心に固定したBMI270の鉛直Z軸ヨーで目標到達・進行不足を確認します。初回は必ず車輪を浮かせ、左右モーターが逆転すること、Z軸が期待方向へ増えること、残ヨー20度以下で220 RPMへ減速することを順に確認してください。

USB接続だけの試験、静止音響試験、DoA座標校正、車輪を浮かせたアクチュエータ試験、接地試験の順序は[ReSpeaker統合設計の「安全な導入・検証順」](../../docs/firmware/RESPEAKER_INTEGRATION.md#8-安全な導入検証順)に従ってください。目標RPMは20 kHz PWMの基本値へ変換し、現在の代表エンコーダ値は診断にだけ使用します。

### CPU0状態LED

`task_think`が青LEDと緑LEDを所有し、赤LEDはCPU1が所有します。

| LED表示 | 意味 |
|---|---|
| 青: 500 msごとに反転 | USB link待ち・safe stop |
| 青: 1秒ごとに100 ms点灯 | 停止して音を聴取中 |
| 青: 125 msごとに反転 | 目標方向へservo整定中 |
| 青: 点灯 | 1000 msの短距離移動中 |
| 青: 250 msごとに反転 | settleまたはcooldown中 |
| 緑: 消灯 | 未学習。SW1を2秒長押しして学習開始 |
| 緑: 500 msごとに点滅 | 学習中 |
| 緑: 100 msごとに点滅 | 5見本収集済み。SW1を2秒長押しして保存 |
| 緑: 点灯 | MRAMへ保存済み |
| 緑: 1秒ごとに短い2回点滅 | MRAM初期化・保存・読出しエラー |
| 青: 回数点滅 | CPU0 fault。青の点滅回数がfault番号 |
| 赤: 500 msごとに反転 | CPU1正常heartbeat |
| 赤: 約50 msごとに反転 | CPU1でFSP/driverエラー発生 |

CPU0 fault番号は、1回がタスク生成／開始、2回がIPC初期化、3回がIPC送信、4回が思考目標タイムアウト、5回がUSB初期化、6回が目標共有失敗です。

CPU1をデバッグして、次の変数をLive Watchへ追加すると確認しやすくなります。

| 変数 | 内容 |
|---|---|
| `g_servo_pulse_us[0..3]` | 各サーボの現在の指令幅 |
| `g_drive_left_duty_permille` | 左モーターの符号付きPWM指令（正値はRPWM、負値はLPWM） |
| `g_drive_right_duty_permille` | 右モーターの符号付きPWM指令（正値はRPWM、負値はLPWM） |
| `g_encoder_left_encoder_count` / `g_encoder_right_encoder_count` | 左右代表モーターの4逓倍累積カウント（前進正、後進負） |
| `g_encoder_left_rpm_x10` / `g_encoder_right_rpm_x10` | 左右代表モーター推定RPMの10倍（前進正、100 ms更新） |
| `g_actuator_service_last_error` | 最後のFSPエラー |
| `g_actuator_service_fault_flags` | タイムアウト、緊急停止、指令制限、IPC・ドライバー異常フラグ |

CPU0では次の変数をLive Watchへ追加します。

| 変数 | 内容 |
|---|---|
| `g_task_think_state` | link待ち、聴取、servo整定、短距離移動、settle、cooldown、fault |
| `g_task_think_cycle_count` | 思考タスクの実行回数 |
| `g_task_think_observation_sequence` | 思考で最後に使用した音響観測sequence |
| `g_task_think_observation_watchdog_ms` | 同じ観測sequenceが続いた時間。300 msでlink無効、未成立時は`UINT32_MAX` |
| `g_task_think_fault_flags` | CPU0でラッチした異常ビット |
| `g_task_command_sequence` | 最終IPCシーケンス番号 |
| `g_task_command_send_count` | 正常にcommitした指令数 |
| `g_task_command_last_error` | 最後のIPC FSPエラー |
| `g_task_acoustic_link_usb_configured` | HCDC deviceの列挙状態 |
| `g_task_acoustic_link_hello_received` | 音響frontendとのHELLO成立 |
| `g_task_acoustic_link_frame_count` | 正常な音響frame受信数 |
| `g_task_acoustic_link_crc_error_count` | CRC異常frame数 |
| `g_task_acoustic_link_observation_age_ms` | 最新音響観測からの経過時間 |
| `g_task_acoustic_link_observation` | 最新DoA、level、peak、VAD、状態、flags |
| `g_task_sensor_initialized` / `g_task_sensor_last_error` | センサーハブの初期化完了状態と、現在のFSPエラー。正常時は`1` / `0` |
| `g_task_sensor_tof_distance_mm[0..2]` | 左・中央・右ToFの最新距離[mm] |
| `g_task_sensor_tof_result[0..2]` / `g_task_sensor_tof_range_status[0..2]` | ToF読出し結果とVL53L1XのRange Status。`tof_result`が`0`かつRange Statusが`9`なら有効な測距値 |
| `g_task_sensor_failure_kind` / `g_task_sensor_failure_device` / `g_task_sensor_failure_stage` / `g_task_sensor_failure_channel` | 直近の公開周期における失敗箇所。成功周期では`NONE`、`NONE`、`NONE`、`-1`へ戻る |

回転方向やカウント符号が意図と逆なら、モーターのOUT1/OUT2、A/B相、またはソフトウェアの符号規約のいずれか一つだけを入れ替えます。

## 主な実装箇所

| ファイル | 役割 |
|---|---|
| `../common/acoustic_protocol.h/.c` | XIAO ESP32S3とCPU0で共有するUSB音響protocol |
| `common/ipc_message.h` | 両CPU共通のコマンド型と32 bit通信形式 |
| `SoundExplorationRover_CPU0/src/main.c` | CPU1起動とCPU0タスク群の起動 |
| `SoundExplorationRover_CPU0/src/tasks/task_registry.c` | CPU0タスク登録配列、全タスクの生成・開始、失敗時の逆順解放 |
| `SoundExplorationRover_CPU0/src/tasks/task_acoustic_link.c` | HCDC event、frame検証、最新音響snapshot |
| `SoundExplorationRover_CPU0/src/services/acoustic_ai_lab_link.c` | J11 USB CDCによるAI学習・推論品質snapshot、特徴量/profile読出し、学習操作 |
| `../../software/acoustic-ai-lab/` | JSON Lines記録とブラウザ可視化を行うPC側品質評価ツール。Rover Monitor/UDPとは独立 |
| `SoundExplorationRover_CPU0/src/tasks/task_think.c` | 音響snapshot取得、目標展開、青・緑LED、CPU0 faultラッチ |
| `SoundExplorationRover_CPU0/src/control/sound_follow_controller.c` | 停止聴取型の音源追従状態機械 |
| `SoundExplorationRover_CPU0/src/tasks/task_command.c` | 最新目標の共有、期限監視、全アクチュエータのIPC一括送信 |
| `SoundExplorationRover_CPU0/src/ipc/` | CPU0側IPCコマンド送信・CPU1状態受信 |
| `SoundExplorationRover_CPU1/src/tasks/`、`src/services/` | CPU1初期化、周期処理、フェイルセーフ |
| `SoundExplorationRover_CPU1/src/drivers/` | DCモーター、エンコーダ、サーボとFSP/BSP APIの呼び出し |
| `SoundExplorationRover_CPU1/src/ipc/` | CPU1側IPCコマンド受信・コミット・実出力状態送信 |
| `SoundExplorationRover_CPU0/src/config/` | CPU0周期、優先度、音量/DoA/移動、LED設定 |
| `SoundExplorationRover_CPU1/src/config/` | ピン、周期、制限値、安全タイムアウト |
| `SoundExplorationRover_CPU1/src/hal_entry.c` | 生成コードとユーザーアプリを接続する入口 |

### CPU0タスク一覧

`task_registry.c`は独立カーネルタスクではなく、`usermain()`から呼ばれるタスク登録・初期化モジュールです。登録配列に生成・開始・解放APIを追加するだけでCPU0タスクを増減できます。全リソースの生成に成功してから各タスクを開始し、失敗時は生成済みリソースを登録の逆順で解放します。`usermain()`はμT-Kernel初期タスクの実行コンテキストを永久休止させ、登録された3タスクを別々のスタックと優先度で実行します。

| ファイル | カーネルタスク | 優先度 | 周期・待ち | 役割 |
|---|---|---:|---|---|
| `task_command.c` | `task_command_entry` | 6 | 50 ms | CPU1へ6出力分の最新目標を送信 |
| `task_acoustic_link.c` | `task_acoustic_link_entry` | 8 | 1 ms poll | USB HCDCの音響観測受信、frame parser、テレメトリ送信 |
| `task_think.c` | `task_think_entry` | 10 | 100 ms | 音源追従目標、LED、faultラッチ |

IPCの構成方法と、受信側に割り込み・コールバックが必要という条件は、Renesasの[Getting Started with IPC on Dual Core MCU](https://www.renesas.com/en/document/apn/getting-started-ipc-dual-core-mcu)に準拠しています。

## TFLM推論環境（Phase 1）

CPU0のC++17参照カーネル版TFLMと静的96 KiBアリーナを導入済みです。
取得元・ライセンス・C API・ビルド条件は [common/tflm/README.md](common/tflm/README.md)、
学習環境とホスト推論検証は [acoustic-trainer](../../software/acoustic-trainer/README.md) を参照してください。
モデルの学習と推論タスクの走行経路への接続はPhase 2で実装します。

## CPU1タイミング是正（Phase 0-A 第1段）

駆動周期をイベントフラグで起床する構成へ移し、RPM・ランプ・指令期限を実時間化しました。
ビルドとホスト試験は成功、実機の停止・周期確認は未実施です。
監視変数と必須の測定項目は [検証記録](../../docs/firmware/validation/README.md) を参照してください。
