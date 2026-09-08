# RA8P1 レイヤー構成

RA8P1のユーザーコードは、責務に基づいて次の5層へ分ける。ディレクトリ名には層番号を含めず、役割を表す名称を使う。

```text
L4 tasks
    ↓
L3 control
    ↓
L2 services
    ↓
L1 drivers
    ↓
L0 platform
    ↓
Renesas FSP / ハードウェア
```

| 層 | ディレクトリ | 責務 | 代表例 |
|---|---|---|---|
| L0 | `platform/` | FSPに近い汎用的な基盤機能 | CPU0 `i2c_bus` |
| L1 | `drivers/` | 個別デバイス、PWM、GPIO、IRQの操作 | `vl53l1x`、`bmi270`、`tca9548a`、`bts7960`、`servo`、`encoder` |
| L2 | `services/` | 複数ドライバの統合、安全状態、変換・校正 | `sensor_hub`、`drive_service`、`actuator_service` |
| L3 | `control/` | センサー状態から走行目標を決定 | `sound_follow_controller`、`obstacle_avoidance_controller` |
| L4 | `tasks/` | μT-Kernelタスクの生成、周期、待機、上位層の呼出し | `task_acoustic_link`、`task_sensor`、`task_think`、`task_command` |

`ipc/` と `config/` は横断的な役割を持つ。`ipc/` はCPU間通信の変換・送受信を、`config/` は用途別の調整値と基板上の役割を保持する。FSPのピン多重化設定は引き続きSolutionを正とし、`pin_config.h` はアプリケーション側の役割対応だけを定義する。

CPU0では、`sensor_hub` がTCA9548Aのチャネルを選択してからToFまたはIMUドライバを呼ぶ。各センサードライバはTCA9548A上の接続チャネルを知らない。CPU1では、`drive_service` がRPM、符号、校正、変化率制限を担当し、`bts7960` は符号付きデューティをPWM出力へ反映するだけにする。

この構成は動作を変えないリファクタリングである。IPCのペイロード、TCA9548Aチャネル割当て、モーター極性、非常停止、指令タイムアウト、センサー回復、および音源追従の振る舞いは既存の契約を維持する。
