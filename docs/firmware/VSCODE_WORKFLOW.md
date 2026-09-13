# VS Code統合開発手順

この文書は、EK-RA8P1のCPU0/CPU1とReSpeaker上のXIAO ESP32S3を、一つのVS Codeワークスペースからビルド、書き込み、デバッグする手順を定義します。

FSPのマルチコアSolution、ピン、クロック、スタック構成は引き続きe² studioを正とします。通常のソース編集、ビルド、J-Linkデバッグ、ESP32書き込みはVS Codeから実行できます。

## 1. 開くワークスペース

リポジトリ直下の`SoundExplorationRover.code-workspace`をVS Codeで開きます。大文字小文字を含む別名ではなく、実ファイル名`SoundExplorationRover.code-workspace`を使用します。

初回はVS Codeが提示する推奨拡張機能をインストールします。

- Renesas Platform
- Renesas Build Utilities
- Renesas Debug
- Renesas Memory Usage View
- Renesas Smart Manual
- PlatformIO IDE
- CMake Tools
- C/C++

RenesasサイドバーのQuick InstallでRA向けArm GNU Toolchain、FSP/RA Smart Configurator、RA device support、SEGGER J-Linkが認識されていることを確認します。PlatformIOはサイドバーを一度開き、PlatformIO Coreの初期化を完了させます。

PlatformIOの初回ビルドで`No module named 'intelhex'`が表示された場合は、PlatformIO CoreのPython環境へ一度だけ追加します。

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m pip install intelhex
```

## 2. VS Codeタスク

コマンドパレットの`Tasks: Run Task`から次を実行します。

| タスク | 用途 |
|---|---|
| `Firmware: Build All` | CPU0、CPU1、ESP32を順番にビルドする既定タスク |
| `RA8P1: Generate + Clean Build All` | FSP生成を反映し、CPU0→CPU1の順で完全ビルド |
| `RA8P1: Generate + Clean Build CPU0` | CPU0だけをFSP生成・完全ビルド |
| `RA8P1: Generate + Clean Build CPU1` | CPU1だけをFSP生成・完全ビルド |
| `RA8P1: Fast Build All` | 生成済み`Debug` make metadataを使う差分ビルド |
| `RA8P1: Fast Build CPU0/CPU1` | 対象CPUだけの差分ビルド |
| `ESP32: Build` | PlatformIO/ESP-IDFでXIAOをビルド |
| `ESP32: Upload` | XIAO側USB-Cから書き込み |
| `ESP32: Monitor` | XIAOのUSB CDCを開く。現在はバイナリprotocol専用 |
| `ESP32: Clean` | PlatformIO生成物を削除 |

`Ctrl+Shift+B`は`Firmware: Build All`を実行します。

RA8P1の完全ビルドは、インストール済みe² studioをヘッドレスで呼び出します。これにより既存の`.project`、`.cproject`、FSP Solutionと生成順を維持します。e² studioのGUIを同時に開いている場合は、設定ファイルやワークスペースのロックを避けるため、完全ビルド前にGUIを閉じてください。

macOS版のビルドスクリプトは、e² studio同梱JavaからEquinoxのヘッドレスビルダーを
直接起動します。ネイティブ`Contents/MacOS/e2studio`はヘッドレス指定でもAppKitを初期化し、
サンドボックスやCIからの起動時に`RegisterApplication`でSIGABRTするため、完全ビルド用途では
直接実行しません。VS Codeタスクまたは`.vscode/scripts/Invoke-RaBuild.sh`を使用してください。

`Fast Build`は、最後の完全ビルド以降に新しいソースファイルやFSPモジュールを追加していない場合だけ使います。makefileがない、または新しいソースが列挙されていない場合は`Generate + Clean Build`を実行します。

標準のインストール場所と異なる場合は、VS Codeを起動する前に次を環境変数へ設定します。

| 変数 | 内容 |
|---|---|
| `E2STUDIO_HOME` | `eclipse`フォルダーを含むe² studioのインストールルート |
| `ARM_GCC_TOOLCHAIN_PATH` | `arm-none-eabi-gcc.exe`を含む`bin`フォルダー |
| `PLATFORMIO_CORE_DIR` | PlatformIO Coreデータディレクトリ |

## 3. RA8P1の書き込みとデバッグ

EK-RA8P1のJ10デバッグUSBをPCへ接続し、実行と停止ビューから次の構成を選びます。

| 構成 | 用途 |
|---|---|
| `RA8P1 CPU0 (J-Link OB)` | CPU0 ELFの書き込みとCPU0デバッグ |
| `RA8P1 CPU1 Program / Standalone Debug (J-Link OB)` | CPU1 ELFの書き込み、またはCPU1単独接続 |
| `RA8P1 CPU1 Attach (J-Link OB)` | CPU0起動後にCPU1へhot-plug接続 |

Renesas Debugがプログラムを尋ねたら次を指定します。

- CPU0: `firmware/ra8p1/SoundExplorationRover_CPU0/Debug/SoundExplorationRover_CPU0.elf`
- CPU1: `firmware/ra8p1/SoundExplorationRover_CPU1/Debug/SoundExplorationRover_CPU1.elf`

両CPUを更新・デバッグする順序は次です。

1. `RA8P1: Generate + Clean Build All`を実行します。
2. `RA8P1 CPU1 Program / Standalone Debug`を開始し、CPU1 ELFを書き込んでセッションを終了します。
3. `RA8P1 CPU0`を開始し、CPU0 ELFを書き込んでCPU0を起動します。
4. CPU1も同時に観測する場合は、CPU0セッションを維持したまま`RA8P1 CPU1 Attach`を開始します。

CPU1 Attachは`hotPlug`接続なので、CPU1のフラッシュを書き換えません。先にCPU1 Programを行ってください。最初はCPU0とCPU1を個別に確認し、両デバッグセッションの同時利用はEK-RA8P1/J-Link OBの接続が安定している状態で行います。

μT-KernelはFSPが直接認識するRTOSではないため、Renesas DebugのRTOS integrationは有効にしていません。タスク状態は既存のLive Watch用グローバル変数とカーネルオブジェクトを使って確認します。

### J10のオンボードJ-Linkが見つからない場合

J-LinkのProbe Selectionに`No probes connected via USB`と表示される場合は、CPU0/CPU1のlaunch設定より前に、WindowsがJ10上のRA4M2 Debug MCUを認識していない。TCP/IP接続は選ばず、次を確認します。

1. e² studioとVS Codeの全デバッグセッションを終了します。
2. J7とモーター電源を一時的に外し、J10をデータ対応USB-CケーブルでPC本体のUSBポートへ直結します。
3. EK-RA8P1のLED5が点滅を止め、橙点灯になることを確認します。
4. デバイスマネージャーの`ポート (COMとLPT)`に`JLink CDC UART Port`が現れることを確認します。
5. Debug On-boardのジャンパーを確認します。J6は2-3、J8は1-2、J9は2-3、J29は1-2/3-4/5-6/7-8です。
6. LED5が点滅したままなら、別のデータ対応ケーブルと別のPC直結ポートを試し、SEGGER J-Link USB driverを修復します。
7. J-Linkを認識した後に主電源とJ7を戻し、CPU0 multicore launchを開始します。

主電源が入っていても、J10のUSB data経路、RA4M2 Debug MCU、ジャンパーが正しくなければJ-Link probeは列挙されません。

`JLinkLog.log`に、Renesas DebugのJ-Link DLLと`JLinkGUIServer.exe`の異なるversionが並び、`Failed to open DLL`と記録される場合はSEGGER softwareのversion混在です。e² studioとVS CodeのRenesas Debugは、Renesas Platformが管理する同じDebug Componentを参照するため、個別の`launch.json`へJ-Linkのversionを記述しません。本開発環境ではSEGGER J-Link Software Pack 9.14aを9.42と並列インストールし、RA向けRenesas Debug Componentの`JLinkARM.dll`をFSP 6.4.0同梱の9.14aへ戻した上で、公式Software Packの`JLinkGUIServer.exe`も9.14aへ統一しています。9.42 DLLは復旧用バックアップとして保持します。

## 4. XIAO ESP32S3のビルドと書き込み

PlatformIO環境は`firmware/esp32s3/platformio.ini`に定義しています。

- board: `seeed_xiao_esp32s3`
- framework: ESP-IDF
- PlatformIO Espressif32: `6.13.0`
- ESP-IDF: 5.5.3系
- source directory: `main`

`ESP32: Upload`を行うときは、XIAO側USB-CをEK-RA8P1 J7から外し、PCへ直接接続します。書き込み後はPCから外し、EK-RA8P1 J7へ戻します。PCとEK-RA8P1の二つのUSB hostへ同時接続しないでください。

PlatformIOがCOMポートを検出した後、`Connecting...`と`Write timeout`で停止した場合は、実行中のTinyUSB CDCは認識できている一方、XIAOがROMダウンロードモードへ入っていません。XIAO本体の`BOOT`を押したままXIAO本体の`RESET`を押して離し、最後に`BOOT`を離してから`ESP32: Upload`を再実行します。ReSpeaker基板側のXVF3800用`RESET`ではなく、XIAO上の小さいボタンを使用します。代わりに、XIAO側USB-Cを外し、XIAOの`BOOT`を押したままPCへ接続してから`BOOT`を離しても移行できます。

現在のXIAO USB CDCはRA8P1向けバイナリprotocol専用です。`ESP32: Monitor`に通常のテキストログが表示されないこと、バイナリを文字として表示すると読めないことは正常です。

ソースレベルのESP32デバッグにはESP-Prog、J-Link、CMSIS-DAPなどの外部JTAGプローブが必要です。プローブが確定するまでは、PlatformIO設定へ`debug_tool`を固定しません。

## 5. FSP、ピン、スタックを変更するとき

マルチコア全体のピン所有関係を含む変更は、次の順序を維持します。

1. e² studioで`SoundExplorationRover` Solutionを開きます。
2. SolutionのPin Configurationを正として編集します。
3. CPU0、CPU1の順にGenerate Project Contentを実行します。
4. e² studioを閉じます。
5. VS Codeへ戻り、`RA8P1: Generate + Clean Build All`を実行します。

CPUローカルの`configuration.xml`はVS CodeからRA Smart Configuratorで開くこともできますが、SolutionとCPU0/CPU1のピン設定を重複させないでください。

## 6. 生成物

次はローカル生成物としてGit管理しません。

- CPU0/CPU1の`Debug`、`Release`
- PlatformIOの`.pio`
- ESP-IDFの`managed_components`、`sdkconfig`、環境別の`sdkconfig.*`（`sdkconfig.defaults`を除く）
- VS Codeから起動するe² studioのヘッドレスワークスペース
- ELF、SREC、MAP、ログ

FSPが生成する`ra`、`ra_cfg`、`ra_gen`と、`.project`、`.cproject`、`configuration.xml`は再現性のためGit管理します。
