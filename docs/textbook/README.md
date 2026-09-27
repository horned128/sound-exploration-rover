# Sound Exploration ROVer（SEROV）開発・設計 教科書

本ディレクトリ（`docs/textbook/`）は、TRONプログラミングコンテスト2026参加プロジェクト「**Sound Exploration ROVer（SEROV）**」の全貌を体系的に学び、開発・保守・拡張・検証を行うための総合技術教科書（Textbook）です。

---

## 📖 教科書の歩き方（Learning Path）

本教科書は、初心者から熟練の組み込みエンジニアまでが段階的に理解を深められるよう、10の章とエッジAI特化サブ教科書、および3つのインタラクティブWeb教材で構成されています。

```text
【入門・全体像】
  ├─ 01_introduction_and_overview.md       : プロジェクトの目的、背景、開発哲学
  ├─ 02_hardware_and_mechanics.md          : ロッカーボギー機構、アクチュエータ、電源・配線
  └─ 03_system_architecture.md             : 4プロセッサの責務分担と5層＋AIレイヤー設計

【ファームウェア・制御の中核】
  ├─ 04_rtos_and_multicore.md              : μT-Kernel 3.0、RA8P1デュアルコア、task_infer隔離設計
  ├─ 05_interprocessor_communication.md    : 32-bit IPC FIFO、6出力アトミックコミット、USB CDC v2
  ├─ 06_acoustic_dsp_and_edge_ai.md        : XVF3800、Log-Mel、TFLM実機検証知見、音源ナビゲーション
  └─ 07_sensor_and_autonomy.md             : ToF 3眼、TFLM制御MLP、5層多層防御安全アーキテクチャ

【検証・運用・実践】
  ├─ 08_software_ecosystem_and_sim.md      : rover-monitor、control-sim、rover-sim、acoustic-trainer
  ├─ 09_development_workflow_and_build.md  : VS Code、Fast Build、J-Link一括Flash、FSP生成サイクル
  └─ 10_testing_tuning_and_troubleshooting.md: 実機校正値、Live Watch、最新実機トラブルシューティング

【★特化サブ教科書: エッジAI & 音響DSP】
  └─ edgeai/                              : 音響DSP・Log-Mel・int8量子化・TFLM・現場学習（全7章）
```

### 🌐 統合Webリーダー
- [**教科書総合ポータル・統合リーダー (`index.html`)**](index.html): 全体目次、章要約、重要用語集（Glossary）、およびMarked.jsによるメイン10章＋エッジAI特化編全7章のMarkdownインライン閲覧・検索・Mermaid図レンダリング対応統合UI。

---

## 📚 目次一覧

| 章 | タイトル | 主な学習内容 |
|:---:|---|---|
| **第1章** | [プロジェクト概要とミッション](01_introduction_and_overview.md) | コンテスト趣旨、音源探査の意義、モノレポ構成、基本設計哲学 |
| **第2章** | [ハードウェア機構とメカニカル設計](02_hardware_and_mechanics.md) | ロッカーボギー式車体、独自3Dパーツ、モータ・サーボ仕様、電源分離とスターGND |
| **第3章** | [4プロセッサ協調アーキテクチャ](03_system_architecture.md) | XVF3800 / ESP32-S3 / RA8P1 CPU0 / CPU1の責務分離、5層＋AIレイヤー構造、ピン所有権 |
| **第4章** | [μT-Kernel 3.0とデュアルコア制御](04_rtos_and_multicore.md) | Cortex-M85/M33異種マルチコア、SRAM分離、BSP上書き構成、タスク優先度・task_infer隔離設計 |
| **第5章** | [プロセッサ間通信プロトコル](05_interprocessor_communication.md) | 4段32-bit IPC FIFO、アトミックコミット、USB CDC v2、状態ビットパッキング、音源ナビ診断 |
| **第6章** | [音響信号処理・エッジAI・音源ナビゲーション](06_acoustic_dsp_and_edge_ai.md) | XVF3800、16kHz Log-Mel、音響TFLM実機検証知見、192D DSP要約ベースライン、スピンターン |
| **第7章** | [センサ統合・TFLM制御MLP・多層防御自律走行](07_sensor_and_autonomy.md) | TCA9548A、SCL端子移設、TFLM制御MLP（Policy Distillation）、5層多層防御、スタック脱出 |
| **第8章** | [ソフトウェアエコシステムとシミュレーション](08_software_ecosystem_and_sim.md) | rover-monitor（.jsonlログ）、control-sim（ASan/UBSanテスト）、rover-sim、acoustic-trainer |
| **第9章** | [開発ワークフロー・ビルド・FSP生成手順](09_development_workflow_and_build.md) | VS Code、Invoke-RaBuild（高速並列ビルド & J-Link一括書き込み）、FSP生成7ステップ |
| **第10章** | [実機キャリブレーション・テスト・トラブルシューティング](10_testing_tuning_and_troubleshooting.md) | エンコーダ実測（702 counts/rev）、Live Watch必携シンボル、最新実機検証知見トラブル対策 |

---

## 🎯 前提知識と対象読者
- **C言語・組み込みプログラミング**: ポインタ、構造体、ビット演算、volatile修飾子、リングバッファの理解。
- **リアルタイムOS（RTOS）の基本**: タスク、優先度、コンテキストスイッチ、ミューテックス、イベントフラグ、周期ハンドラの概念。
- **マイコン周辺機能**: GPIO、PWM（タイマ）、I2C、I2S、SPI、USB、UART、外部割り込み（IRQ）。
- **Python環境**: `uv` パッケージマネージャによるスクリプト実行、データ可視化。
