# ZeusCar ROS2 Project

## プロジェクト概要
ZeusCarはROS2 Jazzyベースのロボットカープロジェクトです。

## 開発目標

### ホストPC側（本環境）
**`reference/zeuscar-project/pc_control/`と同等の機能をROS2 Jazzyで実現する。**

実装する機能:
- **Publisherノード**: キーボードから制御コマンドを入力し、ROS2トピックに送信
- **制御コマンド**: 11種類のモーター制御（前進、後退、左右旋回、斜め移動、回転、停止）
- **通信**: ROS2トピック「topic」でRaspberry Piと連携

```
入力コマンド一覧:
0: FORWARD (前進)      1: BACKWARD (後退)
2: LEFT (左旋回)       3: RIGHT (右旋回)
4: LEFTFORWARD (左前)  5: RIGHTFORWARD (右前)
6: LEFTBACKWARD (左後) 7: RIGHTBACKWARD (右後)
8: TURNLEFT (左回転)   9: TURNRIGHT (右回転)
10: STOP (停止)
```

### Raspberry Pi側
**`reference/zeuscar-project/ros2/`と同等の機能をROS2 Jazzyで実現する。**

実装する機能:
- **Subscriberノード**: ROS2トピックからコマンドを受信し、Arduinoにシリアル送信
- **シリアル通信**: /dev/ttyACM0, 9600bps
- **Arduino連携**: メカナムホイール制御

### システム構成
```
[Host PC]              [Raspberry Pi 4]         [Arduino Uno R3]
Publisher Node  ──────> Subscriber Node ──────> Motor Driver
(キーボード入力)  ROS2    (シリアル転送)   Serial  (メカナム制御)
                 topic
```

### ベースプロジェクトとの違い
| 項目 | ベース（reference/） | 本プロジェクト |
|------|---------------------|---------------|
| ROS2バージョン | Humble Hawksbill | **Jazzy Jalisco** |
| 対応OS | Ubuntu 20.04/22.04 | **Ubuntu 24.04** |
| 依存パッケージ | ros-humble-* | **ros-jazzy-*** |

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
- **gitにプッシュする前に機密情報（トークン、パスワード、APIキー等）が含まれていないか確認する**
- **gitへのプッシュは必ずユーザーに確認を取ってから行う**

## ドキュメント作成ルール（最重要）
**本プロジェクトの最大の目的は、ROS2を使ったロボット開発のための技術書を執筆するための情報蓄積である。**
システム完成よりもドキュメントの充実を優先すること。

### 作業の進め方
- **実装作業を行う際は、必ず先にセットアップガイドに手順を記載してから実行する**
- **全てのコマンド、設定、手順を漏れなく記録する**
- 作業と同時並行でドキュメントを更新する（後回しにしない）

### 索引（docs/README.md）
- **ドキュメントを追加・変更した場合は必ず索引を更新する**

### セットアップガイド（docs/setup-guide.md）
- **環境構築に必要な全ての手順を記載する**
- 実行したコマンドは全て記録する
- インストールしたパッケージは全て記録する
- 設定変更は全て記録する
- 手順の順序を明確にする
- 期待される出力例を記載する
- なぜその手順が必要かの説明を加える

### トラブルシューティング（docs/troubleshooting.md）
- **作業中にエラーに遭遇したら、必ずトラブルシューティングに記載する**
- **エラーはノウハウとして価値がある。解決法と併せて必ず記録する**
- **発生した全てのエラーを記録する**（ビルドエラー、実行時エラー、通信エラー等）
- エラーメッセージは正確に記載する
- 発生状況を詳細に記載する
- 原因が判明した場合は記載する
- **解決方法は必ず記載する**
- 参考にしたリソースがあれば記載する
- エラーを無視して進めない。必ず記録してから次に進む

## 重要な制約
- **`reference/`ディレクトリ内のファイルは参照専用です。絶対に変更・編集・削除してはいけません。**

## リファレンス
- ベースプロジェクト: `reference/zeuscar-project/`

## 言語設定
- Claude Codeとの対話は日本語で行う
