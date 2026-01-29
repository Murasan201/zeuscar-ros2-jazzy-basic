# バックログ - Raspberry Pi

## 概要
本ドキュメントは、ZeusCar ROS2プロジェクトのRaspberry Pi担当範囲のバックログを管理する。

## 対象範囲
- Ubuntu 24.04 LTS環境構築
- ROS2 Jazzy環境構築
- ROS2パッケージの移行・開発
- Raspberry Pi上での動作確認

---

## バックログ一覧

### 凡例
| ステータス | 説明 |
|-----------|------|
| [ ] | 未着手 |
| [x] | 完了 |
| [!] | ブロック中 |

---

## 1. 環境構築

### 1.1 OS環境
| ID | タスク | ステータス | 備考 |
|----|--------|-----------|------|
| RPI-001 | Ubuntu 24.04 LTSのインストール確認 | [x] | 既にインストール済み |
| RPI-002 | システムアップデート | [x] | apt-get update/upgrade完了 |
| RPI-003 | 必要なシステムパッケージのインストール | [x] | build-essential, cmake, git等 |

### 1.2 ROS2環境
| ID | タスク | ステータス | 備考 |
|----|--------|-----------|------|
| RPI-010 | ROS2 Jazzyのインストール | [x] | ros-jazzy-desktop, ros-dev-tools |
| RPI-011 | ROS2環境変数の設定 | [x] | ~/.bashrcに追加 |
| RPI-012 | colconのインストール | [x] | python3-colcon-common-extensions |
| RPI-013 | rosdepのセットアップ | [x] | rosdep init/update完了 |
| RPI-014 | ROS2動作確認 | [x] | ros2 topic list確認済み |

### 1.3 開発環境
| ID | タスク | ステータス | 備考 |
|----|--------|-----------|------|
| RPI-020 | Gitのセットアップ | [x] | 完了 |
| RPI-021 | Python開発環境の確認 | [x] | python3-pip, python3-dev |
| RPI-022 | C++開発環境の確認 | [x] | g++, cmake |

---

## 2. パッケージ移行

### 2.1 zeuscar_robot_package
| ID | タスク | ステータス | 備考 |
|----|--------|-----------|------|
| RPI-100 | パッケージ構造の作成 | [x] | src/zeuscar_robot_package/ |
| RPI-101 | package.xmlの作成（Jazzy対応） | [x] | rclpy, std_msgs依存 |
| RPI-102 | setup.pyの作成 | [x] | entry_points設定 |
| RPI-103 | setup.cfgの作成 | [x] | install_scripts設定 |
| RPI-104 | subscriber.pyの移行 | [x] | docstring追加 |
| RPI-105 | pyserialの依存関係追加 | [x] | システムパッケージ使用 |
| RPI-106 | パッケージビルド確認 | [x] | colcon build成功 |
| RPI-107 | ノード起動確認 | [x] | シリアルエラー確認済み |

---

## 3. ハードウェア連携

### 3.1 Arduino連携
| ID | タスク | ステータス | 備考 |
|----|--------|-----------|------|
| RPI-200 | シリアルポートの確認 | [ ] | /dev/ttyACM0 |
| RPI-201 | シリアル通信テスト | [ ] | |
| RPI-202 | 移動コマンド送信テスト | [ ] | |

---

## 4. 統合テスト

| ID | タスク | ステータス | 備考 |
|----|--------|-----------|------|
| RPI-300 | パッケージのビルド確認 | [x] | zeuscar_robot_package |
| RPI-301 | Subscriberノード動作確認 | [ ] | |
| RPI-302 | 全移動コマンドの動作確認 | [ ] | 11コマンド |
| RPI-303 | PC側との通信テスト | [ ] | 遠隔操作確認 |

---

## 5. ドキュメント

| ID | タスク | ステータス | 備考 |
|----|--------|-----------|------|
| RPI-400 | セットアップガイドの完成 | [ ] | |
| RPI-401 | トラブルシューティングの更新 | [ ] | |
| RPI-402 | 動作確認手順書の作成 | [ ] | |

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2026-01-25 | 初版作成 |
| 2026-01-25 | RPI-001〜003, RPI-021〜022 完了 |
| 2026-01-28 | RPI-010〜014 完了（ROS2 Jazzy環境構築完了） |
| 2026-01-29 | RPI-100〜107 完了（zeuscar_robot_package移行完了） |
| 2026-01-29 | robot_description, sllidar_ros2を対象外に変更（遠隔操作のみのため） |
