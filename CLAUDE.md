# ZeusCar ROS2 Project

## プロジェクト概要
ZeusCarはROS2 Jazzyベースのロボットカープロジェクトです。

## 技術スタック
- **ROS2**: Jazzy Jalisco
- **言語**: Python 3 / C++
- **プラットフォーム**: Raspberry Pi (Linux)

## ディレクトリ構造
```
zeuscar-ros2-jazzy-basic/
├── src/                  # ROS2パッケージ
├── launch/               # Launchファイル
├── config/               # 設定ファイル
├── reference/            # 参照用ファイル（読み取り専用）
│   └── zeuscar-project/  # ベースプロジェクト
├── docs/                 # ドキュメント
├── .claude/              # Claude Code設定
│   └── settings.json
├── CLAUDE.md             # Claude Codeルールファイル
└── LICENSE
```

## ビルド方法
```bash
# ワークスペースのビルド
colcon build

# 特定パッケージのビルド
colcon build --packages-select <package_name>

# 環境のセットアップ
source install/setup.bash
```

## コーディング規約
- Pythonノード: PEP 8に準拠
- C++ノード: ROS2 C++スタイルガイドに準拠
- コミットメッセージ: 日本語可、変更内容を簡潔に記述

## 開発ルール
- 新しいノードを作成する際は対応するLaunchファイルも作成する
- パラメータはYAMLファイルで管理する
- テストコードを可能な限り追加する
- **本プロジェクトのドキュメントは`docs/`ディレクトリに集約する**
- **一旦作業が終わったらコミットを行う**

## 重要な制約
- **`reference/`ディレクトリ内のファイルは参照専用です。絶対に変更・編集・削除してはいけません。**

## リファレンス
- ベースプロジェクト: `reference/zeuscar-project/`

## 言語設定
- Claude Codeとの対話は日本語で行う
