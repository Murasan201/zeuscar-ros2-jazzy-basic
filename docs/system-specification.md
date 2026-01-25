# システム仕様書

## 1. 概要

### 1.1 目的
本書は、zeuscar-ros2-jazzy-basicプロジェクトのシステム仕様を定義する。

### 1.2 スコープ
- ROS2 Jazzy環境でのロボット制御システム
- リファレンスプロジェクト（zeuscar-project）の機能を維持

## 2. システムアーキテクチャ

### 2.1 全体構成
```
┌─────────────────┐
│   Host PC       │
│  (Publisher)    │
└────────┬────────┘
         │ ROS2 Topic: "topic"
         │ (std_msgs/String)
         ▼
┌─────────────────┐
│ Raspberry Pi 4  │
│  (Subscriber)   │
│  (sllidar_node) │
└────────┬────────┘
         │ Serial: /dev/ttyACM0
         │ 9600 baud
         ▼
┌─────────────────┐
│  Arduino Uno    │
│   R3            │
└────────┬────────┘
         │ PWM制御
         ▼
┌─────────────────┐
│  DCモーター×4   │
└─────────────────┘
```

### 2.2 ソフトウェア環境
| 項目 | 仕様 |
|------|------|
| OS | Ubuntu 24.04.3 LTS (Noble Numbat) |
| ROS2 | Jazzy Jalisco |
| Python | 3.x |
| C++ | C++17 |

## 3. ディレクトリ構成

### 3.1 プロジェクト全体
```
zeuscar-ros2-jazzy-basic/
├── src/
│   ├── zeuscar_robot_package/    # ロボット制御パッケージ
│   ├── robot_description/        # ロボット定義パッケージ
│   └── sllidar_ros2/             # LiDARドライバパッケージ
├── docs/                         # ドキュメント
├── reference/                    # 参照用（Git管理外）
├── .claude/
│   └── settings.json
├── .gitignore
├── CLAUDE.md
└── LICENSE
```

### 3.2 zeuscar_robot_package
```
zeuscar_robot_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── zeuscar_robot_package
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── zeuscar_robot_package/
    ├── __init__.py
    └── subscriber.py
```

### 3.3 robot_description
```
robot_description/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── display.launch.py
└── urdf/
    ├── robot.xacro
    └── robot.urdf
```

### 3.4 sllidar_ros2
```
sllidar_ros2/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── sllidar_a1_launch.py
│   ├── sllidar_a2m7_launch.py
│   ├── sllidar_a2m8_launch.py
│   ├── sllidar_a2m12_launch.py
│   ├── sllidar_a3_launch.py
│   ├── sllidar_c1_launch.py
│   ├── sllidar_s1_launch.py
│   ├── sllidar_s1_tcp_launch.py
│   ├── sllidar_s2_launch.py
│   ├── sllidar_s2e_launch.py
│   ├── sllidar_s3_launch.py
│   ├── sllidar_t1_launch.py
│   └── view_sllidar_*_launch.py
├── rviz/
│   └── sllidar_ros2.rviz
├── scripts/
│   ├── create_udev_rules.sh
│   ├── delete_udev_rules.sh
│   └── rplidar.rules
├── sdk/
│   ├── include/
│   └── src/
└── src/
    ├── sllidar_node.cpp
    └── sllidar_client.cpp
```

## 4. パッケージ仕様

### 4.1 zeuscar_robot_package

#### 4.1.1 基本情報
| 項目 | 値 |
|------|-----|
| パッケージ名 | zeuscar_robot_package |
| ビルドタイプ | ament_python |
| バージョン | 0.0.0 |
| ライセンス | MIT |

#### 4.1.2 依存関係
| 種別 | パッケージ |
|------|-----------|
| 実行依存 | rclpy |
| 実行依存 | std_msgs |
| 実行依存 | pyserial |
| テスト依存 | ament_copyright |
| テスト依存 | ament_flake8 |
| テスト依存 | ament_pep257 |
| テスト依存 | python3-pytest |

#### 4.1.3 エントリポイント
| 名前 | モジュール |
|------|-----------|
| subscriber_node | zeuscar_robot_package.subscriber:main |

### 4.2 robot_description

#### 4.2.1 基本情報
| 項目 | 値 |
|------|-----|
| パッケージ名 | robot_description |
| ビルドタイプ | ament_cmake |
| バージョン | 0.0.0 |
| ライセンス | Apache 2.0 |

#### 4.2.2 依存関係
| 種別 | パッケージ |
|------|-----------|
| ビルドツール | ament_cmake |
| 実行依存 | robot_state_publisher |
| 実行依存 | xacro |
| 実行依存 | rviz2 |

#### 4.2.3 URDFフレーム構成
| リンク名 | 説明 |
|---------|------|
| base_link | ベースフレーム |
| laser | LiDARフレーム |

#### 4.2.4 LiDAR取り付け位置
| パラメータ | 値 | 単位 |
|-----------|-----|------|
| x | -0.017 | m |
| y | 0.0165 | m |
| z | 0.21 | m |
| roll | 0 | rad |
| pitch | 0 | rad |
| yaw | 0 | rad |

### 4.3 sllidar_ros2

#### 4.3.1 基本情報
| 項目 | 値 |
|------|-----|
| パッケージ名 | sllidar_ros2 |
| ビルドタイプ | ament_cmake |
| バージョン | 1.0.1 |
| ライセンス | BSD |

#### 4.3.2 依存関係
| 種別 | パッケージ |
|------|-----------|
| ビルドツール | ament_cmake |
| 依存 | rclcpp |
| 依存 | std_srvs |
| 依存 | sensor_msgs |

#### 4.3.3 対応LiDARモデル
- RPLIDAR A1
- RPLIDAR A2 (M7/M8/M12)
- RPLIDAR A3
- RPLIDAR C1
- RPLIDAR S1/S2/S2E/S3
- RPLIDAR T1

## 5. ノード仕様

### 5.1 minimal_subscriber

#### 5.1.1 基本情報
| 項目 | 値 |
|------|-----|
| ノード名 | minimal_subscriber |
| パッケージ | zeuscar_robot_package |
| 言語 | Python |

#### 5.1.2 サブスクライブトピック
| トピック名 | メッセージ型 | QoS |
|-----------|-------------|-----|
| topic | std_msgs/String | 10 |

#### 5.1.3 シリアル通信設定
| パラメータ | 値 |
|-----------|-----|
| ポート | /dev/ttyACM0 |
| ボーレート | 9600 |
| 初期化待機 | 2秒 |

### 5.2 sllidar_node

#### 5.2.1 基本情報
| 項目 | 値 |
|------|-----|
| ノード名 | sllidar_node |
| パッケージ | sllidar_ros2 |
| 言語 | C++ |

#### 5.2.2 パブリッシュトピック
| トピック名 | メッセージ型 |
|-----------|-------------|
| /scan | sensor_msgs/LaserScan |

#### 5.2.3 パラメータ
| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| channel_type | serial | 接続タイプ |
| serial_port | /dev/ttyUSB0 | シリアルポート |
| serial_baudrate | 115200 | ボーレート |
| frame_id | laser | フレームID |
| inverted | false | スキャン反転 |
| angle_compensate | true | 角度補正 |
| scan_mode | Sensitivity | スキャンモード |

## 6. 通信仕様

### 6.1 ROS2トピック通信

#### 6.1.1 移動コマンドトピック
| 項目 | 値 |
|------|-----|
| トピック名 | topic |
| メッセージ型 | std_msgs/String |
| 方向 | Publisher → Subscriber |

### 6.2 シリアル通信

#### 6.2.1 通信設定
| 項目 | 値 |
|------|-----|
| ポート | /dev/ttyACM0 |
| ボーレート | 9600 |
| データビット | 8 |
| パリティ | なし |
| ストップビット | 1 |

#### 6.2.2 コマンドフォーマット
| コマンド | 動作 | モーター出力 [0,1,2,3] |
|---------|------|----------------------|
| FORWARD | 前進 | [+,+,+,+] |
| BACKWARD | 後退 | [-,-,-,-] |
| LEFT | 左平行移動 | [-,+,-,+] |
| RIGHT | 右平行移動 | [+,-,+,-] |
| LEFTFORWARD | 左前進 | [0,+,0,+] |
| RIGHTFORWARD | 右前進 | [+,0,+,0] |
| LEFTBACKWARD | 左後退 | [-,0,-,0] |
| RIGHTBACKWARD | 右後退 | [0,-,0,-] |
| TURNLEFT | 左旋回 | [-,+,+,-] |
| TURNRIGHT | 右旋回 | [+,-,-,+] |
| STOP | 停止 | [0,0,0,0] |

## 7. ハードウェアインターフェース

### 7.1 モーター配置
```
  [0]--|||--[1]    (前輪)
   |         |
   |         |
   |         |
   |         |
  [3]-------[2]    (後輪)
```

### 7.2 Arduinoピン配置
| モーター | PWMピン | 方向ピン |
|---------|--------|---------|
| 0 (左前) | 3 | A3 |
| 1 (右前) | 4 | A2 |
| 2 (右後) | 5 | A1 |
| 3 (左後) | 6 | A0 |

### 7.3 モーター制御パラメータ
| パラメータ | 値 |
|-----------|-----|
| PWM最小値 | 28/255 |
| デフォルトパワー | 80/255 |
| PWM最大値 | 255/255 |
| フェード時間 | 100ms |

## 8. Launchファイル仕様

### 8.1 display.launch.py
ロボットの可視化用Launchファイル

| 起動ノード | パッケージ |
|-----------|-----------|
| robot_state_publisher | robot_state_publisher |
| rviz2 | rviz2 |

### 8.2 sllidar_*_launch.py
LiDARノード起動用Launchファイル（モデル別）

| 起動ノード | パッケージ |
|-----------|-----------|
| sllidar_node | sllidar_ros2 |

## 9. ビルド手順

### 9.1 依存関係のインストール
```bash
cd ~/zeuscar-ros2-jazzy-basic
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 9.2 ビルド
```bash
colcon build --symlink-install
```

### 9.3 環境設定
```bash
source install/setup.bash
```

## 10. 実行方法

### 10.1 Subscriberノード起動
```bash
ros2 run zeuscar_robot_package subscriber_node
```

### 10.2 LiDARノード起動
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

### 10.3 可視化
```bash
ros2 launch robot_description display.launch.py
```
