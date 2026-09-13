# ドキュメント

このディレクトリには、リポジトリ全体で参照する詳細設計書、開発手順、コーディング規約を配置します。各コンポーネント直下の`README.md`は概要、使い方、詳細文書への入口として扱います。

## 構成

- [`firmware/ARCHITECTURE.md`](firmware/ARCHITECTURE.md): CPU0、CPU1、XIAO ESP32S3の全体設計とRA8P1の5層構成
- [`firmware/archify/SEROV_ARCHITECTURE.html`](firmware/archify/SEROV_ARCHITECTURE.html): 現行コードのCPU0、CPU1、XIAO ESP32S3、監視ツール間の構造を示すインタラクティブ図
- [`firmware/SENSOR_AUTONOMY.md`](firmware/SENSOR_AUTONOMY.md): BMI270、TCA9548A、VL53L1X×3の接続、ルールベース走行、FSP生成、試験手順
- [`firmware/edgeai/`](firmware/edgeai/): 音源識別AIの設計・共有契約
- [`firmware/validation/`](firmware/validation/): タイミング、生存性、基盤実装の検証記録
- [`hardware/`](hardware/): 車体・駆動系・エンコーダの測定結果と採用値
- `program-plan-2026_ja.pdf`: プロジェクト計画書

部品の仕様・購入情報は、引き続き対象コンポーネント配下の`hardware/*/spec/`へ置きます。現在のアクチュエータ仕様書は`hardware/actuator/spec/`です。
