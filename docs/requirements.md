# 要件定義書

## 1. プロジェクト概要

### 1.1 プロジェクト名
zeuscar-ros2-jazzy-basic

### 1.2 目的
既存プロジェクト「zeuscar-project」の機能を維持したまま、OSおよびROS2のバージョンのみを更新した新規プロジェクトを作成する。

### 1.3 背景
- 既存プロジェクトはUbuntu 22.04 + ROS2 Humbleで構築されている
- 長期サポート（LTS）版への移行によりセキュリティと安定性を確保する

## 2. 対象環境

### 2.1 ハードウェア
| 項目 | 仕様 | 備考 |
|------|------|------|
| メインボード | Raspberry Pi 4 | 変更不可 |
| マイコン | Arduino Uno R3 | 変更不可 |
| センサー | SLAMTEC LiDAR | 変更不可 |
| モーター | DC モーター × 4 | 変更不可 |

### 2.2 ソフトウェア
| 項目 | 移行前 | 移行後 |
|------|--------|--------|
| OS | Ubuntu 22.04 | Ubuntu 24.04.3 LTS (Noble Numbat) |
| ROS2 | Humble Hawksbill | Jazzy Jalisco (LTS) |

## 3. 変更範囲

### 3.1 実施する変更
- [ ] OSをUbuntu 24.04系に更新する
- [ ] ROS2をJazzyに更新する
- [ ] 新規GitHubリポジトリ「zeuscar-ros2-jazzy-basic」を作成する

### 3.2 実施しない変更
- ハードウェア構成の変更
- 機能仕様の変更
- ノード構成・通信仕様の変更
- 制御ロジックの変更
- インターフェース仕様の変更

## 4. 機能要件

### 4.1 維持すべき機能

#### 4.1.1 ロボット制御機能
リファレンスプロジェクトと同等の移動制御機能を提供すること。

| コマンド | 動作 |
|---------|------|
| FORWARD | 前進 |
| BACKWARD | 後退 |
| LEFT | 左平行移動 |
| RIGHT | 右平行移動 |
| LEFTFORWARD | 左前進 |
| RIGHTFORWARD | 右前進 |
| LEFTBACKWARD | 左後退 |
| RIGHTBACKWARD | 右後退 |
| TURNLEFT | 左旋回 |
| TURNRIGHT | 右旋回 |
| STOP | 停止 |

#### 4.1.2 通信機能
- ROS2トピックによるコマンド受信
- シリアル通信によるArduinoへのコマンド送信

#### 4.1.3 センサー機能
- LiDARセンサーからのスキャンデータ取得
- ROS2トピックへのデータ配信

### 4.2 ROS2パッケージ構成
リファレンスプロジェクトと同等のパッケージ構成とすること。

| パッケージ名 | 機能 |
|-------------|------|
| zeuscar_robot_package | ロボット制御（Subscriber） |
| robot_description | URDF/Xacroロボット定義 |
| sllidar_ros2 | LiDARドライバ |

### 4.3 ノード構成
| ノード名 | 役割 |
|---------|------|
| minimal_subscriber | ROSコマンド受信・Arduinoへ転送 |
| sllidar_node | LiDARデータ配信 |

### 4.4 トピック構成
| トピック名 | メッセージ型 | 用途 |
|-----------|-------------|------|
| topic | std_msgs/String | 移動コマンド |
| /scan | sensor_msgs/LaserScan | LiDARスキャンデータ |

## 5. 非機能要件

### 5.1 互換性
- 既存のArduinoファームウェアとの互換性を維持すること
- シリアル通信仕様（9600baud、/dev/ttyACM0）を維持すること

### 5.2 性能
- リファレンスプロジェクトと同等の応答性能を維持すること

## 6. 成果物

| 成果物 | 説明 |
|--------|------|
| GitHubリポジトリ | zeuscar-ros2-jazzy-basic |
| ROS2パッケージ | Ubuntu 24.04 + ROS2 Jazzy対応 |
| ドキュメント | 要件定義書、設計書等 |

## 7. 受入条件

- [ ] Ubuntu 24.04 + ROS2 Jazzy環境で動作すること
- [ ] 既存プロジェクトと同等の機能が動作すること
- [ ] 全ての移動コマンドが正常に動作すること
- [ ] LiDARセンサーからデータ取得が可能であること
