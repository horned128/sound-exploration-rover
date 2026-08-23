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
│   └── ra8p1/                  EK-RA8P1用e² studioワークスペース
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

`firmware/ra8p1/`をe² studioのワークスペースとして使用します。

DS3225MGとJGA25-370をCPU0から指令し、CPU1でPWM・エンコーダ制御するサンプルと配線は、[EK-RA8P1 アクチュエータ制御サンプル](firmware/ra8p1/README.md)を参照してください。

| プロジェクト | 内容 |
|---|---|
| `SoundExplorationRover` | マルチコア構成をまとめるソリューション |
| `SoundExplorationRover_CPU0` | CPU0向けプロジェクト |
| `SoundExplorationRover_CPU1` | CPU1向けプロジェクト |

共有する`.project`、`.cproject`、FSP設定、ソースコードはGit管理の対象です。`.metadata/`、`Debug/`、`Release/`、`build/`、起動設定、ログ、ELFなどのローカル生成物は管理しません。
