# 開発ジャーナル

## 2026-01-25

### 作業内容

#### プロジェクト初期設定
- [x] `.claude/settings.json` 作成（Claude Code許可ツール設定）
- [x] `CLAUDE.md` 作成（プロジェクトルールファイル）
- [x] `.gitignore` 作成（reference/等を除外）
- [x] `reference/` ディレクトリ作成
- [x] `docs/` ディレクトリ作成

#### ドキュメント作成
- [x] `docs/requirements.md` - 要件定義書
- [x] `docs/system-specification.md` - システム仕様書
- [x] `docs/setup-guide.md` - セットアップガイド
- [x] `docs/troubleshooting.md` - トラブルシューティング
- [x] `docs/backlog-raspberry-pi.md` - Raspberry Piバックログ
- [x] `docs/README.md` - ドキュメント索引

#### ルール設定
- ドキュメントは`docs/`に集約
- `reference/`は参照専用（変更禁止）
- 作業完了後はコミット
- プッシュ前に機密情報チェック
- プッシュ前にユーザー確認必須
- セットアップ手順は全て記録
- エラーと解決方法は全て記録
- ドキュメント追加時は索引更新

#### 環境構築（バックログ1.1 OS環境）
- [x] RPI-001: Ubuntu 24.04 LTSのインストール確認
- [x] RPI-002: システムアップデート（apt-get update/upgrade）
- [x] RPI-003: 必要なシステムパッケージのインストール
  - build-essential, cmake, git, wget, curl, vim, python3-pip, software-properties-common
- [x] RPI-021: Python開発環境の確認
- [x] RPI-022: C++開発環境の確認

#### Git設定
- ユーザー名: Murasan201
- メール: shinf0330@gmail.com
- リモートURLにトークン設定済み

### コミット履歴
1. `73a492a` - プロジェクト初期設定と要件定義書を追加
2. `16d47d4` - 機密情報確認ルールを追加
3. `aa80a33` - システム仕様書を追加
4. `00874a6` - プッシュ前の確認ルールを追加
5. `a49c389` - セットアップガイドとトラブルシューティングを追加
6. `4c11f8a` - Raspberry Piバックログを追加
7. `264770b` - ドキュメント索引を追加
8. `bad9c4f` - OS環境セットアップ完了（RPI-001〜003, RPI-021〜022）

---

## 2026-01-28

### 作業内容

#### ドキュメント更新
- [x] セットアップガイドに開発環境確認手順を追加（1.4〜1.6）
  - 1.4 Gitのセットアップ
  - 1.5 Python開発環境の確認
  - 1.6 C++開発環境の確認（LiDARドライバーで必要な理由を説明）
- [x] セットアップガイドにROS2インストール手順を詳細に記載（セクション2〜3）
  - 2.1 ロケール設定
  - 2.2 リポジトリの追加（GPGキー、ソースリスト）
  - 2.3 ROS2パッケージのインストール
  - 2.4 環境設定
  - 2.5 colconのインストール
  - 2.6 rosdepのセットアップ
  - 2.7 ROS2動作確認
  - 3.1〜3.3 開発環境のセットアップ

#### ルールファイル更新
- [x] CLAUDE.md更新
  - ドキュメント作成ルールを「最重要」に格上げ
  - 「技術書執筆のための情報蓄積が最大の目的」を明記
  - 作業前にセットアップガイドに手順を記載するルールを追加
  - エラー遭遇時のトラブルシューティング記載を必須化

#### 設定ファイル更新
- [x] `.claude/settings.local.json`更新
  - 全Bashコマンド許可（`Bash(*)`）に変更
  - 作業効率化のため認証プロンプトを削減

#### 環境構築（バックログ1.2 ROS2環境）- 完了
- [x] ROS2リポジトリの追加
  - Universeリポジトリ有効化
  - ROS2 GPGキーダウンロード
  - ROS2ソースリスト追加（/etc/apt/sources.list.d/ros2.list）
- [x] RPI-010: ROS2 Jazzyのインストール
  - `ros-jazzy-desktop`インストール完了
  - `ros-dev-tools`インストール完了
- [x] RPI-011: ROS2環境変数の設定
  - `source /opt/ros/jazzy/setup.bash`を~/.bashrcに追加
- [x] RPI-012: colconのインストール
  - `python3-colcon-common-extensions`インストール完了
  - colcon補完を~/.bashrcに追加
- [x] RPI-013: rosdepのセットアップ
  - `sudo rosdep init`実行完了
  - `rosdep update`実行完了
- [x] RPI-014: ROS2動作確認
  - `ros2 topic list`で動作確認済み

#### トラブルシューティング記録
- [x] ENV-001: ROS2リポジトリ追加時にファイルが空になる問題
  - 原因: シェルのコマンド置換とsudo teeの組み合わせ
  - 解決: `sudo bash -c`で全体をroot権限で実行
- [x] ENV-002: dpkgロックエラー（unattended-upgrades）
  - 原因: 自動セキュリティアップデートがバックグラウンドで実行中
  - 解決: プロセス強制終了とロック解除で対応

### 現在の状態
- ROS2 Jazzy環境構築完了
- バックログ1.2の全タスク（RPI-010〜014）完了

---

## 2026-01-29

### 作業内容

#### zeuscar_robot_packageの移行（RPI-100〜107）完了
- [x] RPI-100: パッケージ構造の作成
  - src/zeuscar_robot_package/ディレクトリ構造
  - resource/, test/, zeuscar_robot_package/サブディレクトリ
- [x] RPI-101: package.xmlの作成（Jazzy対応）
  - format="3"、ament_python
  - 依存関係: rclpy, std_msgs
- [x] RPI-102: setup.pyの作成
  - entry_points設定（subscriber_node）
- [x] RPI-103: setup.cfgの作成
  - install_scripts設定
- [x] RPI-104: subscriber.pyの移行
  - docstring追加（pep257対応）
- [x] RPI-105: pyserialの依存関係追加
  - python3-serialはシステムパッケージとして使用
- [x] RPI-106: パッケージビルド確認
  - `colcon build --paths src/*`で成功
- [x] RPI-107: ノード起動確認
  - シリアルポートエラー（Permission denied）確認済み

#### セットアップガイド更新
- [x] セクション4「ROS2パッケージの作成」を追加
  - 4.1 ワークスペースの準備
  - 4.2 zeuscar_robot_package（Pythonパッケージ）
    - パッケージ構造、package.xml、setup.py、setup.cfg
    - subscriber.pyの解説
    - テストファイル
    - ビルドと動作確認

#### トラブルシューティング記録
- [x] BUILD-001: colconで重複パッケージ名エラー
  - 原因: reference/ディレクトリ内の同名パッケージ
  - 解決: `--paths src/*`オプションの使用

#### プロジェクトスコープの見直し
- 本プロジェクトは「遠隔操作」までを対象とする
- システム構成を再確認：
  ```
  [Host PC] -- ROS2 topic --> [Raspberry Pi] -- Serial --> [Arduino]
  Publisher                   Subscriber                   Motor Controller
  ```
- 以下のパッケージは対象外に変更：
  - robot_description（URDF/Xacro）- RViz可視化は不要
  - sllidar_ros2（LiDARドライバー）- LiDAR対応は将来の拡張
- 誤って移行したパッケージを削除：
  - `rm -rf src/robot_description src/sllidar_ros2`
- ビルドディレクトリをクリーンアップ：
  - `rm -rf build/ install/ log/`
- パッケージ再ビルド確認済み

#### セットアップガイドの修正
- [x] 1.6節: C++環境の理由からLiDAR記載を削除
- [x] 3.3節: 追加ROS2パッケージをコメントアウト（将来用）
- [x] 4節概要: zeuscar_robot_packageのみに変更
- [x] 4.3〜4.4節: robot_description, sllidar_ros2を削除
- [x] 5.2〜5.3節: LiDAR関連を削除、Arduino udevのみ残す
- [x] 6.3節: LiDAR動作確認を削除
- [x] 変更履歴を更新

#### バックログの修正
- [x] セクション2.2, 2.3（robot_description, sllidar_ros2）を削除
- [x] セクション3.2（LiDAR連携）を削除
- [x] セクション4（統合テスト）からLiDAR関連タスクを削除

### 教訓
- **セットアップガイドは「実行した手順の記録」であるべき**
- 将来の予定を先に書くべきではない
- 参照プロジェクトのスコープを最初に確認すべき

### 現在の状態
- zeuscar_robot_package移行完了
- バックログ2.1の全タスク（RPI-100〜107）完了
- 次のタスク: ハードウェア連携（Arduino）

### 現在のsrc/構成
```
src/
└── zeuscar_robot_package/  （Subscriberノード - PC側からのコマンドをArduinoに転送）
```

---

## 次回作業開始時にやること

### 1. ROS2環境の確認
```bash
# ROS2環境が正しく読み込まれているか確認
source ~/.bashrc
printenv ROS_DISTRO  # "jazzy"と表示されればOK
ros2 topic list      # /parameter_events, /rosoutが表示されればOK
```

### 2. バックログ確認
パッケージ移行は完了。次のフェーズは **ハードウェア連携（Arduino）** です。

| ID | タスク |
|----|--------|
| RPI-200〜202 | Arduino連携 |
| RPI-300〜303 | 統合テスト |

### 3. 次のタスク: Arduino連携テスト

Arduinoを接続してシリアル通信をテスト:
```bash
ros2 run zeuscar_robot_package subscriber_node
```

パッケージビルド確認:
```bash
colcon build --paths src/*
source install/setup.bash
ros2 pkg list
```

### 4. セットアップガイドに手順を記載
実装作業の前に、必ずセットアップガイドに手順を記載すること。

### 5. コミット
作業完了後、変更をコミットする。
