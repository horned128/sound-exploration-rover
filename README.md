# Sound Exploration ROVer（SEROV）

TRONプログラミングコンテスト2026に向けて開発する、音源探索ローバー **Sound Exploration ROVer（SEROV）** のモノレポです。

- [Sound Exploration ROVer（SEROV）](#sound-exploration-roverserov)
  - [取得方法](#取得方法)
  - [プロジェクト構成](#プロジェクト構成)
  - [ハードウェア](#ハードウェア)
    - [Papaya追加機構](#papaya追加機構)
    - [アクチュエータ](#アクチュエータ)
  - [ファームウェア](#ファームウェア)

## 取得方法

このリポジトリは、GitサブモジュールとGit LFSを含めて次のように取得します。

```bash
git clone --recurse-submodules <repository-url>
cd sound-exploration-rover
git lfs pull
```

既存のクローンでGitサブモジュールを取得する場合は、次を実行します。

```bash
git submodule update --init --recursive
git lfs pull
```

STEPファイルとSTLファイルはGit LFSの対象です。Gitサブモジュールは上流ブランチを直接追跡せず、親リポジトリが指定するコミットへ固定します。

## プロジェクト構成

```text
sound-exploration-rover/
├── docs/                       ドキュメント類
├── firmware/
│   ├── common/                 MCU間で共有する通信protocol
│   ├── ra8p1/                  EK-RA8P1用e² studioワークスペース
│   └── esp32s3/ XIAO ESP32S3用ESP-IDF project
└── hardware/
    ├── actuator/               使用アクチュエータの仕様
    ├── papaya-addon/
    │   ├── step/               編集用のSTEPデータ
    │   └── stl/                造形用のSTLデータ
    └── papaya-pathfinder/      Papaya Pathfinderの3Dモデル（Gitサブモジュール）
```

`hardware/papaya-pathfinder/`は、Gitサブモジュールを初期化すると作成されます。

## ハードウェア

ベース車体には、オープンソースのロッカーボギー型ローバーPapaya Pathfinderを採用しています。公開されている3Dモデルを造形し、EK-RA8P1とマイクアレイを搭載するための追加機構を独自に設計しています。

### Papaya追加機構

`hardware/papaya-addon/`には、EK-RA8P1とマイクアレイをPapaya車体へ搭載する追加機構を格納しています。

- `step/`には、形状編集、干渉確認、組付け設計に使用するデータを格納します。
- `stl/`には、3Dプリント用メッシュを格納します。
- 同じ部品は、STEPとSTLでファイル名を一致させます。

上流のPapaya Pathfinderが提供する標準車体と3Dモデルは、`hardware/papaya-pathfinder/`から参照します。独自設計は`hardware/papaya-addon/`へ分離して管理します。

### アクチュエータ

使用するモーターとサーボの購入仕様、販売ページ、参考仕様は、[アクチュエータ仕様一覧](hardware/actuator/README.md)にまとめています。

## ファームウェア

通常の編集、ビルド、書き込み、デバッグは、リポジトリ直下の`SoundExplorationRover.code-workspace`をVS Codeで開いて行います。FSP Solution、ピン、クロック、スタックを変更するときは`firmware/ra8p1/`をe² studioのワークスペースとして使用します。

詳細な設計書・開発手順・コーディング規約は[docs](docs/README.md)へ集約しています。VS Codeの初期設定とCPU0/CPU1/XIAOの操作は[VS Code統合開発手順](docs/firmware/VSCODE_WORKFLOW.md)を参照してください。全体の責務、CPU0/CPU1タスク、IPC、アクチュエータは[ローバー ファームウェア設計書](docs/firmware/ARCHITECTURE.md)、ReSpeaker/XIAOのUSB接続、protocol、DoA校正、音源追従の検証順は[ReSpeaker統合設計](docs/firmware/RESPEAKER_INTEGRATION.md)を参照してください。従来の配線とアクチュエータ単体確認は[EK-RA8P1 アクチュエータ制御](firmware/ra8p1/README.md)に残しています。

| プロジェクト | 内容 |
|---|---|
| `SoundExplorationRover` | マルチコア構成をまとめるソリューション |
| `SoundExplorationRover_CPU0` | CPU0向けプロジェクト |
| `SoundExplorationRover_CPU1` | CPU1向けプロジェクト |
| `esp32s3` | XVF3800連携、USB CDC音響frontend |

共有する`.project`、`.cproject`、FSP設定、ソースコードはGit管理の対象です。`.metadata/`、`Debug/`、`Release/`、`build/`、起動設定、ログ、ELFなどのローカル生成物は管理しません。
