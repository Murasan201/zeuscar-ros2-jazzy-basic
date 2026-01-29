# セットアップガイド

## 概要
本ドキュメントは、ZeusCar ROS2プロジェクトの環境構築手順を記載する。
Ubuntu 24.04 LTS + ROS2 Jazzy環境の標準セットアップ手順書として使用する。

## 対象環境

| 項目 | バージョン |
|------|-----------|
| ハードウェア | Raspberry Pi 4 |
| OS | Ubuntu 24.04.3 LTS (Noble Numbat) |
| ROS2 | Jazzy Jalisco |

---

## 1. OSのインストール

### 1.1 Ubuntu 24.04のインストール

#### 1.1.1 対象ハードウェア
- Raspberry Pi 4 Model B

#### 1.1.2 OSイメージのダウンロード
Ubuntu公式サイトからRaspberry Pi用のUbuntu 24.04 LTS (Noble Numbat) Server imageをダウンロードする。

- URL: https://ubuntu.com/download/raspberry-pi

#### 1.1.3 OSイメージの書き込み
Raspberry Pi Imagerを使用してmicroSDカードにイメージを書き込む。

#### 1.1.4 インストール確認
```bash
# OS情報の確認
cat /etc/os-release

# 期待される出力
# PRETTY_NAME="Ubuntu 24.04 LTS"
# VERSION="24.04 LTS (Noble Numbat)"
# VERSION_CODENAME=noble
```

```bash
# カーネル情報の確認
uname -a

# 期待される出力例
# Linux pi4-ros2-basic 6.8.0-1031-raspi ... aarch64 GNU/Linux
```

```bash
# Raspberry Piモデルの確認
cat /proc/device-tree/model

# 期待される出力例
# Raspberry Pi 4 Model B Rev 1.5
```

### 1.2 システムアップデート

システムを最新の状態に更新する。

```bash
# パッケージリストの更新
sudo apt-get update

# パッケージのアップグレード
sudo apt-get upgrade -y
```

### 1.3 必要なシステムパッケージのインストール

開発に必要な基本パッケージをインストールする。

```bash
# 基本開発ツール
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    python3-pip

# ソフトウェアプロパティ（リポジトリ追加用）
sudo apt-get install -y software-properties-common
```

### 1.4 Gitのセットアップ

Gitの初期設定を行う。

#### 1.4.1 Gitのインストール確認

```bash
# Gitバージョンの確認
git --version

# 期待される出力例
# git version 2.43.0
```

#### 1.4.2 Gitユーザー情報の設定

コミット時に使用するユーザー名とメールアドレスを設定する。

```bash
# ユーザー名の設定
git config --global user.name "Your Name"

# メールアドレスの設定
git config --global user.email "your.email@example.com"

# 設定の確認
git config --global --list
```

#### 1.4.3 デフォルトブランチ名の設定（任意）

```bash
# デフォルトブランチ名をmainに設定
git config --global init.defaultBranch main
```

### 1.5 Python開発環境の確認

Python環境が正しくインストールされていることを確認する。

#### 1.5.1 Pythonバージョンの確認

```bash
# Python3バージョンの確認
python3 --version

# 期待される出力例
# Python 3.12.3
```

#### 1.5.2 pipバージョンの確認

```bash
# pip3バージョンの確認
pip3 --version

# 期待される出力例
# pip 24.0 from /usr/lib/python3/dist-packages/pip (python 3.12)
```

#### 1.5.3 Python開発パッケージの確認

```bash
# python3-devパッケージの確認
dpkg -l | grep python3-dev

# インストールされていない場合はインストール
sudo apt-get install -y python3-dev
```

### 1.6 C++開発環境の確認

C++コンパイラとビルドツールが正しくインストールされていることを確認する。

> **Note**: 本プロジェクトでは主にPythonでノードを実装するが、以下の理由でC++環境があると便利である。
> - **一部のROS2依存パッケージ**: ソースからビルドが必要な場合にC++環境を使用
> - **将来の拡張**: C++ノードを追加する場合に備えて環境を準備

#### 1.6.1 GCCバージョンの確認

```bash
# gccバージョンの確認
gcc --version

# 期待される出力例
# gcc (Ubuntu 13.2.0-23ubuntu4) 13.2.0
```

#### 1.6.2 G++バージョンの確認

```bash
# g++バージョンの確認
g++ --version

# 期待される出力例
# g++ (Ubuntu 13.2.0-23ubuntu4) 13.2.0
```

#### 1.6.3 CMakeバージョンの確認

```bash
# cmakeバージョンの確認
cmake --version

# 期待される出力例
# cmake version 3.28.3
```

#### 1.6.4 build-essentialの確認

```bash
# build-essentialパッケージの確認
dpkg -l | grep build-essential

# インストールされていない場合はインストール
sudo apt-get install -y build-essential
```

---

## 2. ROS2 Jazzyのインストール

ROS2 Jazzy JaliscoはUbuntu 24.04 LTS向けのROS2ディストリビューションである。
公式ドキュメント: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 2.1 ロケール設定

ROS2はUTF-8ロケールを必要とする。ロケールが正しく設定されていることを確認する。

```bash
# 現在のロケール確認
locale
```

> **Note**: `LANG=C.UTF-8`または`LANG=en_US.UTF-8`のように、UTF-8が含まれていれば問題ない。
> Ubuntu 24.04のデフォルトでは`C.UTF-8`が設定されていることが多く、そのままで動作する。

UTF-8ロケールが設定されていない場合のみ以下を実行:
```bash
sudo apt-get update && sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 設定確認
locale
```

期待される出力例:
```
LANG=C.UTF-8          # または LANG=en_US.UTF-8
LANGUAGE=
LC_CTYPE="C.UTF-8"    # UTF-8が含まれていればOK
...
```

### 2.2 リポジトリの追加

ROS2パッケージをインストールするために、ROS2のaptリポジトリを追加する。

#### 2.2.1 Universeリポジトリの有効化

```bash
# software-properties-commonがインストールされていることを確認
sudo apt-get install -y software-properties-common

# Universeリポジトリを有効化
sudo add-apt-repository universe
```

#### 2.2.2 ROS2 GPGキーの追加

```bash
# 必要なツールのインストール
sudo apt-get update && sudo apt-get install -y curl

# ROS2 GPGキーをダウンロードして追加
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

#### 2.2.3 ROS2リポジトリの追加

```bash
# リポジトリをソースリストに追加
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

> **Troubleshooting**: 上記コマンドでファイルが空になる場合は、以下の代替方法を使用する。
> ```bash
> # 代替方法: sudo bash -cで直接書き込み（Raspberry Pi 4 arm64の場合）
> sudo bash -c 'echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list'
> ```
> 詳細は[トラブルシューティング ENV-001](troubleshooting.md#env-001-ros2リポジトリ追加時にファイルが空になる)を参照。

リポジトリが正しく追加されたことを確認:
```bash
cat /etc/apt/sources.list.d/ros2.list

# 期待される出力
# deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main
```

#### 2.2.4 パッケージリストの更新

```bash
# aptパッケージリストを更新
sudo apt-get update

# システムパッケージを最新に（ROS2インストール前に推奨）
sudo apt-get upgrade -y
```

### 2.3 ROS2パッケージのインストール

#### 2.3.1 ROS2 Desktopのインストール

Desktop版には、ROS2コアに加えて可視化ツール（RViz2）やデモパッケージが含まれる。

```bash
# ROS2 Jazzy Desktop版のインストール
sudo apt-get install -y ros-jazzy-desktop
```

> **Note**: インストールには時間がかかる（数分〜十数分程度）。依存パッケージも含めて約2GB程度のダウンロードとなる。

> **Troubleshooting**: `Could not get lock /var/lib/dpkg/lock-frontend`エラーが発生した場合は、
> `unattended-upgrades`（自動セキュリティアップデート）が実行中の可能性がある。
> 完了を待つか、`sudo systemctl stop unattended-upgrades`で一時停止する。
> 詳細は[トラブルシューティング ENV-002](troubleshooting.md#env-002-dpkgロックエラーunattended-upgrades)を参照。

#### 2.3.2 開発ツールのインストール

ROS2パッケージのビルドに必要な開発ツールをインストールする。

```bash
# ROS2開発ツールのインストール
sudo apt-get install -y ros-dev-tools
```

> **Note**: `ros-dev-tools`には以下のツールが含まれる:
> - `rosdep`: ROS2パッケージの依存関係を自動解決
> - `vcstool`: バージョン管理システムツール
> - `colcon`: ROS2ビルドツール（別途拡張パッケージも必要）
> - その他のROS2開発用ユーティリティ

### 2.4 環境設定

#### 2.4.1 ROS2環境のセットアップスクリプト

ROS2を使用するには、セットアップスクリプトをsourceする必要がある。

```bash
# 手動でROS2環境をセットアップ（現在のターミナルのみ）
source /opt/ros/jazzy/setup.bash
```

#### 2.4.2 自動読み込み設定

毎回手動でsourceするのは手間なので、.bashrcに追記して自動化する。

```bash
# .bashrcにROS2セットアップを追加
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 設定を反映
source ~/.bashrc
```

#### 2.4.3 環境変数の確認

```bash
# ROS2関連の環境変数を確認
printenv | grep -i ROS

# 期待される出力例
# ROS_VERSION=2
# ROS_PYTHON_VERSION=3
# ROS_DISTRO=jazzy
```

### 2.5 colconのインストール

colconはROS2のビルドツールである。ros-dev-toolsに含まれているが、追加パッケージもインストールする。

```bash
# colcon関連パッケージのインストール
sudo apt-get install -y python3-colcon-common-extensions
```

#### 2.5.1 colcon補完の設定

colconコマンドのタブ補完を有効にする。

```bash
# colcon補完を.bashrcに追加
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# 設定を反映
source ~/.bashrc
```

### 2.6 rosdepのセットアップ

rosdepはROS2パッケージの依存関係を自動解決するツールである。

#### 2.6.1 rosdepの初期化

```bash
# rosdepの初期化（システム全体で一度だけ実行）
sudo rosdep init
```

> **Note**: 既に初期化済みの場合は「already initialized」と表示される。エラーではない。

#### 2.6.2 rosdepデータベースの更新

```bash
# rosdepデータベースの更新（ユーザーごとに実行）
rosdep update
```

### 2.7 ROS2動作確認

ROS2が正しくインストールされたことを確認する。

#### 2.7.1 バージョン確認

```bash
# ROS2のバージョン確認
ros2 --version

# 期待される出力例
# ros2 0.32.0
```

#### 2.7.2 Talker-Listenerテスト

2つのターミナルを開いて、ROS2のサンプルノードで通信テストを行う。

**ターミナル1（Talker）:**
```bash
# Talkerノードを起動（メッセージを送信）
ros2 run demo_nodes_cpp talker
```

期待される出力:
```
[INFO] [1706000000.000000000] [talker]: Publishing: 'Hello World: 1'
[INFO] [1706000000.000000000] [talker]: Publishing: 'Hello World: 2'
...
```

**ターミナル2（Listener）:**
```bash
# Listenerノードを起動（メッセージを受信）
ros2 run demo_nodes_cpp listener
```

期待される出力:
```
[INFO] [1706000000.000000000] [listener]: I heard: [Hello World: 1]
[INFO] [1706000000.000000000] [listener]: I heard: [Hello World: 2]
...
```

Ctrl+Cで各ノードを終了する。

#### 2.7.3 トピック一覧の確認

```bash
# 現在のトピック一覧を表示
ros2 topic list

# 期待される出力例（ノードが起動していない場合）
# /parameter_events
# /rosout
```

---

## 3. 開発環境のセットアップ

### 3.1 追加の開発パッケージ

プロジェクトで必要となる追加パッケージをインストールする。

```bash
# シリアル通信用（Arduino連携）
sudo apt-get install -y python3-serial

# その他の開発ツール
sudo apt-get install -y python3-pytest python3-pytest-cov
```

### 3.2 ワークスペースの作成

ROS2ワークスペースを作成する。

```bash
# ワークスペースディレクトリの作成
mkdir -p ~/ros2_ws/src

# ワークスペースに移動
cd ~/ros2_ws

# 空のワークスペースをビルド（初期化）
colcon build

# ワークスペース環境のセットアップ
source install/setup.bash
```

#### 3.2.1 ワークスペース環境の自動読み込み

```bash
# ワークスペースのセットアップを.bashrcに追加
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 設定を反映
source ~/.bashrc
```

### 3.3 追加のROS2パッケージ（任意）

将来の拡張で使用する可能性のあるROS2パッケージ。現時点ではインストール不要。

```bash
# TF2（座標変換）- 必要になったらインストール
# sudo apt-get install -y ros-jazzy-tf2-tools

# URDF関連 - ロボットモデル可視化が必要になったらインストール
# sudo apt-get install -y ros-jazzy-urdf ros-jazzy-xacro
# sudo apt-get install -y ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
# sudo apt-get install -y ros-jazzy-robot-state-publisher
```

---

## 4. ROS2パッケージの作成

本プロジェクトでは以下のROS2パッケージを作成する:
- **zeuscar_robot_package**: ロボット制御用Pythonパッケージ（Arduino通信）

> **Note**: 本プロジェクトはPC側からROS2トピック経由でコマンドを受信し、
> Arduinoのメカナムホイールを制御する遠隔操作システムである。

### 4.1 ワークスペースの準備

プロジェクトディレクトリをROS2ワークスペースとして使用する。

```bash
# プロジェクトディレクトリに移動
cd ~/work/zeuscar-ros2-jazzy-basic

# srcディレクトリの作成（パッケージ格納場所）
mkdir -p src
```

### 4.2 zeuscar_robot_package（Pythonパッケージ）

Arduino（モーター制御基板）との通信を担当するPythonパッケージを作成する。

#### 4.2.1 パッケージの概要

| 項目 | 内容 |
|------|------|
| パッケージ名 | zeuscar_robot_package |
| 種類 | ament_python（Pythonパッケージ） |
| 主な機能 | ROS2トピックを受信し、Arduinoにシリアル通信でコマンド送信 |
| 依存関係 | rclpy, std_msgs, pyserial |

#### 4.2.2 パッケージ構造の作成

ROS2 Pythonパッケージの標準構造を作成する。

```bash
# パッケージディレクトリの作成
mkdir -p src/zeuscar_robot_package/zeuscar_robot_package
mkdir -p src/zeuscar_robot_package/resource
mkdir -p src/zeuscar_robot_package/test

# リソースマーカーファイルの作成（ament_indexで必要）
touch src/zeuscar_robot_package/resource/zeuscar_robot_package

# Pythonパッケージの__init__.pyの作成
touch src/zeuscar_robot_package/zeuscar_robot_package/__init__.py
```

最終的なディレクトリ構造:
```
src/zeuscar_robot_package/
├── package.xml              # パッケージメタデータ
├── setup.py                 # Pythonセットアップスクリプト
├── setup.cfg                # セットアップ設定
├── resource/
│   └── zeuscar_robot_package  # リソースマーカー（空ファイル）
├── test/                    # テストファイル
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── zeuscar_robot_package/   # Pythonモジュール
    ├── __init__.py
    └── subscriber.py        # Subscriberノード
```

#### 4.2.3 package.xmlの作成

パッケージのメタデータと依存関係を定義する。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>zeuscar_robot_package</name>
  <version>0.1.0</version>
  <description>ZeusCar robot control package for Arduino communication</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <!-- 実行時依存関係 -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- テスト依存関係 -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

> **Note**: `pyserial`はROS2パッケージではないため、`exec_depend`には含めない。
> システムにpython3-serialをインストールして使用する（セクション3.1参照）。

#### 4.2.4 setup.pyの作成

Pythonパッケージのセットアップスクリプトを作成する。

```python
from setuptools import find_packages, setup

package_name = 'zeuscar_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ZeusCar robot control package for Arduino communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_node = zeuscar_robot_package.subscriber:main',
        ],
    },
)
```

> **Note**: `entry_points`の`console_scripts`で実行可能ノードを定義する。
> 左辺がノード名（`ros2 run`で指定する名前）、右辺が`モジュール名.関数名`。

#### 4.2.5 setup.cfgの作成

Pythonパッケージのインストール先を指定する。

```ini
[develop]
script_dir=$base/lib/zeuscar_robot_package

[install]
install_scripts=$base/lib/zeuscar_robot_package
```

> **Note**: ROS2では実行可能ファイルは`lib/<package_name>/`にインストールされる。

#### 4.2.6 subscriber.pyの作成

Arduinoと通信するサブスクライバノードを作成する。

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time


class MinimalSubscriber(Node):
    """
    文字列メッセージを受信し、それをArduinoへシリアル送信するROS2サブスクライバノード
    """

    def __init__(self):
        # ノード名 'minimal_subscriber' で初期化
        super().__init__('minimal_subscriber')

        # サブスクライバの作成
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

        # シリアルポート設定
        serial_port = '/dev/ttyACM0'  # Arduinoのデフォルトポート
        baud_rate = 9600

        try:
            self.ser = serial.Serial(serial_port, baud_rate)
            # Arduinoリセット直後などの待ち時間を考慮
            time.sleep(2)
            self.get_logger().info(
                f"Serial connection established on {serial_port} at {baud_rate} bps."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None  # ポートが開けなかった場合はNoneにしておく

    def listener_callback(self, msg):
        """
        Subscriberで受信した文字列をArduinoへシリアル送信
        """
        self.get_logger().info(f"I heard: '{msg.data}'")

        # シリアルポートが正常に開いている場合のみ送信
        if self.ser is not None and self.ser.is_open:
            try:
                # 文字列をそのまま送信
                self.ser.write(msg.data.encode())
                self.get_logger().info(f"Sent to Arduino: {msg.data}")

                # 必要があればArduinoの応答を受け取る
                time.sleep(0.1)
                if self.ser.in_waiting > 0:
                    received_data = self.ser.readline().decode(
                        'utf-8', errors='ignore'
                    ).rstrip()
                    self.get_logger().info(f"Received from Arduino: {received_data}")
            except Exception as e:
                self.get_logger().error(f"Serial write/read error: {e}")
        else:
            self.get_logger().warning(
                "Serial port is not open. Unable to send command."
            )


def main(args=None):
    # ROS2の初期化
    rclpy.init(args=args)

    # MinimalSubscriberノードをインスタンス化
    minimal_subscriber = MinimalSubscriber()

    # ノードをスピンさせ、コールバックを待機
    rclpy.spin(minimal_subscriber)

    # ノード終了処理
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 4.2.7 テストファイルの作成

ROS2標準のテストファイルを作成する。

**test/test_copyright.py:**
```python
import pytest
from ament_copyright.main import main


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
```

**test/test_flake8.py:**
```python
import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
```

**test/test_pep257.py:**
```python
import pytest
from ament_pep257.main import main


@pytest.mark.linter
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
```

#### 4.2.8 パッケージのビルド

```bash
# プロジェクトルートに移動
cd ~/work/zeuscar-ros2-jazzy-basic

# パッケージのビルド（src/ディレクトリのみを対象）
colcon build --paths src/*

# 期待される出力
# Starting >>> zeuscar_robot_package
# Finished <<< zeuscar_robot_package [X.XXs]
# Summary: 1 package finished [X.XXs]
```

> **Troubleshooting**: `Duplicate package names not supported`エラーが発生した場合は、
> `reference/`ディレクトリ内に同名のパッケージが存在するためである。
> 必ず`--paths src/*`オプションを使用してビルド対象を限定すること。
> 詳細は[トラブルシューティング BUILD-001](troubleshooting.md#build-001-colconで重複パッケージ名エラー)を参照。

#### 4.2.9 環境のセットアップと動作確認

```bash
# ビルド成果物を環境に読み込み
source install/setup.bash

# パッケージの確認
ros2 pkg list | grep zeuscar

# 期待される出力
# zeuscar_robot_package

# ノードの実行（Arduinoが接続されていない場合はエラーになる）
ros2 run zeuscar_robot_package subscriber_node

# 期待される出力（Arduino未接続時）
# [ERROR] [minimal_subscriber]: Failed to open serial port: [Errno 2] No such file or directory: '/dev/ttyACM0'
```

> **Note**: Arduinoが接続されていない状態では、シリアルポートエラーが発生するが、
> ノード自体は起動し、トピックの受信は可能な状態になる。

---

## 5. ハードウェアセットアップ

### 5.1 Arduinoの接続

Arduinoをシリアル接続でRaspberry Piに接続する。

#### 5.1.1 接続確認

```bash
# USBデバイスの確認
lsusb | grep -i arduino

# シリアルポートの確認
ls -la /dev/ttyACM*

# 期待されるデバイス
# /dev/ttyACM0
```

#### 5.1.2 シリアルポート権限

```bash
# 現在のユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER

# グループ変更を反映（再ログインまたは）
newgrp dialout
```

### 5.2 udevルールの設定（任意）

Arduinoに固定のシンボリックリンクを設定したい場合:

```bash
# Arduinoのベンダー/プロダクトIDを確認
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct"

# udevルールを作成（例: Arduino Uno）
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE:="0777", SYMLINK+="arduino"' | sudo tee /etc/udev/rules.d/99-arduino.rules

# udevの再読み込み
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 6. 動作確認

### 6.1 ROS2の動作確認
（手順を記載予定）

### 6.2 シリアル通信の確認
（手順を記載予定）

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2026-01-25 | 初版作成（テンプレート） |
| 2026-01-25 | 1.1〜1.3 OS環境セットアップ手順を追加 |
| 2026-01-28 | 1.4 Gitセットアップ、1.5 Python環境確認、1.6 C++環境確認を追加 |
| 2026-01-28 | 1.6 C++環境が必要な理由（LiDARドライバー等）の説明を追加 |
| 2026-01-28 | 2. ROS2 Jazzyインストール手順、3. 開発環境セットアップ手順を追加 |
| 2026-01-28 | 2.1, 2.2.3, 2.3.1にトラブルシューティングへの参照を追加 |
| 2026-01-28 | 2.3.2 ros-dev-toolsに含まれるツールの説明を追加 |
| 2026-01-28 | ROS2 Jazzy環境構築完了（実際にインストール実行・動作確認済み） |
| 2026-01-29 | 4. ROS2パッケージの作成セクションを追加 |
| 2026-01-29 | 4.2 zeuscar_robot_package（Pythonパッケージ）の詳細手順を追加 |
| 2026-01-29 | 4.2.8にBUILD-001トラブルシューティングへの参照を追加 |
| 2026-01-29 | 5. ハードウェアセットアップの詳細手順を追加 |
