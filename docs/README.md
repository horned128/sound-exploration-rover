# ドキュメント

このディレクトリには、リポジトリ全体で参照する詳細設計書、開発手順、コーディング規約を配置します。各コンポーネント直下の`README.md`は概要、使い方、詳細文書への入口として扱います。

## 構成

- `firmware/`: ファームウェア全体の設計書、ReSpeaker・センサー統合設計、VS Code開発手順、Cコーディングルール
- [`firmware/archify/SEROV_ARCHITECTURE.html`](firmware/archify/SEROV_ARCHITECTURE.html): 現行コードのCPU0、CPU1、XIAO ESP32S3、監視ツール間の構造を示すインタラクティブ図
- [`firmware/SENSOR_AUTONOMY.md`](firmware/SENSOR_AUTONOMY.md): BMI270、TCA9548A、VL53L1X×3の接続、ルールベース走行、FSP生成、試験手順
- `program-plan-2026_ja.pdf`: プロジェクト計画書

ハードウェアの部品仕様書は、対象コンポーネント配下の`spec/`へ置きます。現在のアクチュエータ仕様書は`hardware/actuator/spec/`です。
