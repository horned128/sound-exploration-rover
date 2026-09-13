# Sound Exploration ROVer（SEROV）開発・設計 教科書

本ディレクトリ（`docs/textbook/`）は、TRONプログラミングコンテスト2026参加プロジェクト「**Sound Exploration ROVer（SEROV）**」の全貌を体系的に学び、開発・保守・拡張・検証を行うための技術教材（Textbook）です。

---

## 📖 教科書の歩き方（Learning Path）

本教科書は、初心者から熟練の組み込みエンジニアまでが段階的に理解を深められるよう、10の章と2つのインタラクティブWeb教材で構成されています。

```text
【入門・全体像】
  ├─ 01_introduction_and_overview.md       : プロジェクトの目的、背景、開発哲学
  ├─ 02_hardware_and_mechanics.md          : ロッカーボギー機構、アクチュエータ、電源・配線
  └─ 03_system_architecture.md             : 4プロセッサの責務分担と5層レイヤー設計

【ファームウェア・制御の中核】
  ├─ 04_rtos_and_multicore.md              : μT-Kernel 3.0、RA8P1デュアルコア、メモリ・タスク設計
  ├─ 05_interprocessor_communication.md    : 32-bit IPC FIFO、6出力アトミックコミット、USB CDC
  ├─ 06_acoustic_dsp_and_edge_ai.md        : XVF3800、Log-Mel抽出、int8量子化、TFLM推論基盤
  └─ 07_sensor_and_autonomy.md             : VL53L1X ToF×3、BMI270、自律回避、生存性監視

【検証・運用・実践】
  ├─ 08_software_ecosystem_and_sim.md      : rover-monitor、control-sim、acoustic-trainer
  ├─ 09_development_workflow_and_build.md  : e² studio / VS Code、FSP設定、macOSビルド運用
  └─ 10_testing_tuning_and_troubleshooting.md: 実機校正値（702 counts/rev）、Live Watch、障害対策

【★特化サブ教科書: エッジAI & 音響DSP】
  └─ edgeai/                              : 音響DSP・Log-Mel・int8量子化・TFLM・現場学習（全7章＋対話型Web教材）
```

### 🌐 インタラクティブWeb教材
- [**教科書総合ポータル・ダッシュボード (`index.html`)**](index.html): 全体目次、章要約、重要用語集（Glossary）、クイックリファレンスを一覧できるポータルUI。
- [**インタラクティブ・システムアーキテクチャ図 (`interactive_system_architecture.html`)**](interactive_system_architecture.html): 4プロセッサ、各種センサ、アクチュエータ、通信バス、電源系統を視覚的・対話的に探索できるSVGビジュアライザ。
- [**音響AIパイプライン・シミュレータ (`edgeai/interactive_audio_pipeline.html`)**](edgeai/interactive_audio_pipeline.html): 音波からLog-Mel・int8・AI推論までの対話型Web教材。

---

## 📚 目次一覧

| 章 | タイトル | 主な学習内容 |
|:---:|---|---|
| **第1章** | [プロジェクト概要とミッション](01_introduction_and_overview.md) | コンテスト趣旨、音源探査の意義、モノレポ構成、基本設計哲学 |
| **第2章** | [ハードウェア機構とメカニカル設計](02_hardware_and_mechanics.md) | ロッカーボギー式車体、独自3Dパーツ、モータ・サーボ仕様、電源分離とスターGND |
| **第3章** | [4プロセッサ協調アーキテクチャ](03_system_architecture.md) | XVF3800 / ESP32-S3 / RA8P1 CPU0 / CPU1の責務分離、5層構造、ピン所有権 |
| **第4章** | [μT-Kernel 3.0とデュアルコア制御](04_rtos_and_multicore.md) | Cortex-M85/M33異種マルチコア、SRAM分離、BSP上書き構成、タスク優先度・周期通知 |
| **第5章** | [プロセッサ間通信プロトコル](05_interprocessor_communication.md) | 4段32-bit IPC FIFO、シーケンス番号によるアトミックコミット、USBバイナリ電文 |
| **第6章** | [音響信号処理・Log-Mel・エッジAI](06_acoustic_dsp_and_edge_ai.md) | ビームフォーミング、DoA/VAD、16kHz Log-Mel 32bin/100fps、int8量子化、TFLM組込み |
| **第7章** | [センサ統合と自律走行アルゴリズム](07_sensor_and_autonomy.md) | TCA9548Aマルチプレクサ、ToF前方3眼幾何学、IMU傾斜保護、200ms更新停止監視 |
| **第8章** | [ソフトウェアエコシステムと検証環境](08_software_ecosystem_and_sim.md) | rover-monitor（Webテレメトリ）、control-sim（ASan/UBSanテスト）、acoustic-trainer |
| **第9章** | [開発ワークフローとビルド手順](09_development_workflow_and_build.md) | VS Codeワークスペース、macOS Equinoxヘッドレスビルド、FSP生成サイクル |
| **第10章** | [実機キャリブレーション・テスト](10_testing_tuning_and_troubleshooting.md) | エンコーダ実測（702 counts/rev）、車体トレッド260mm、SW4設定、Live Watch診断 |

---

## 🎯 前提知識と対象読者
- **C言語・組み込みプログラミング**: ポインタ、構造体、ビット演算、volatile修飾子、リングバッファの理解。
- **リアルタイムOS（RTOS）の基本**: タスク、優先度、コンテキストスイッチ、ミューテックス、イベントフラグ、周期ハンドラの概念。
- **マイコン周辺機能**: GPIO、PWM（タイマ）、I2C、I2S、SPI、USB、UART、外部割り込み（IRQ）。
- **Python環境**: `uv` パッケージマネージャによるスクリプト実行、データ可視化。
