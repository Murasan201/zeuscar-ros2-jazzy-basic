# 開発ジャーナル - ホストPC

## 概要
本ドキュメントは、ZeusCar ROS2プロジェクトのホストPC（Intel NUC）における作業記録を時系列で記録する。

---

## 2026-01-30

### 作業内容

#### ホストPC（Intel NUC）へのUbuntu 24.04インストール

**ホストPCスペック:**
| 項目 | 内容 |
|------|------|
| PC種別 | Intel NUC |
| CPU | Intel Core i7-1255U (12th Gen) |
| メモリ | 8GB |
| 内蔵SSD | WD Blue SN570 1TB NVMe |
| 元OS | Windows 11 + WSL2 (Ubuntu 22.04) |

**インストール方法:**
- USB SSDをインストールメディアとして使用
- Rufusで Ubuntu 24.04 ISOを書き込み（GPT、UEFI）
- 内蔵SSDにクリーンインストール

#### 発生したトラブルと解決

| ID | 問題 | 解決 |
|----|------|------|
| HOST-001 | BIOS/LEGACY BOOT OF UEFI-ONLY MEDIA エラー | ブートメニューで「UEFI:」付きデバイスを選択 |
| HOST-002 | Intel NUC Visual BIOSの操作 | Boot Configuration設定を文書化 |
| HOST-003 | インストール後に再びインストールメニュー表示 | USB抜いて再起動 |
| HOST-004 | ネットワーク接続が切れる | ケーブル抜き差しで復旧 |
| HOST-005 | SSH接続タイムアウト | スリープモードを解除 |
| HOST-006 | SSHホストキー変更エラー | known_hostsから古いキーを削除 |
| HOST-007 | CursorリモートSSHでサーバーダウンロード失敗 | **未解決**（SSL接続エラー） |

#### 完了した設定
- [x] SSHサーバーのインストールと有効化
- [x] SSH接続テスト成功（コマンドラインから）
- [ ] スリープ無効化（推奨）- 未実施
- [ ] CursorリモートSSH接続 - SSL接続エラーで未解決

#### ドキュメント更新
- [x] setup-guide-host-pc.md 大幅更新
- [x] troubleshooting-host-pc.md 新規作成（HOST-001〜007）

---

## 2026-01-31

### 作業内容

#### Wi-Fi接続の安定化

**発生した問題:**
- SSH接続が断続的に切断される
- CursorやTeraTermからの接続が数秒〜数十秒で切れる

**原因:**
- Wi-Fiの省電力機能（Power Save）が有効になっていた

**解決方法:**
```bash
# Wi-Fi接続名の確認
sudo nmcli connection show

# 省電力を無効化
sudo nmcli connection modify "接続名" 802-11-wireless.powersave 2

# 接続を再起動
sudo nmcli connection down "接続名" && sudo nmcli connection up "接続名"
```

#### スリープ無効化

開発用PCとしてスリープを完全に無効化:

```bash
# スリープを無効化
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

#### 発生したトラブルと解決

| ID | 問題 | 解決 |
|----|------|------|
| HOST-008 | Wi-Fi接続でSSHが断続的に切断される | Wi-Fi省電力機能を無効化 |
| HOST-009 | apt/dpkgロックエラー（unattended-upgrades） | プロセス終了を待つか強制終了 |

#### ドキュメント更新
- [x] troubleshooting-host-pc.md にHOST-008、HOST-009を追加
- [x] journal-host-pc.md 新規作成（本ファイル）
- [x] setup-guide-host-pc.md にROS2環境構築手順を追加
  - 6. 開発環境のセットアップ（Git, Python, C++）
  - 7. ROS2 Jazzyのインストール
  - 8. ROS2ワークスペースのセットアップ
  - 9. ROS2ネットワーク設定（マルチマシン構成）

#### ROS2環境構築の開始

**CLAUDE.mdの更新:**
- 開発ゴールを追記
  - ホストPC側: キーボード入力でモーターコマンド(0-10)をパブリッシュするノード
  - Raspberry Pi側: サブスクライバーノードでコマンドを受信しシリアル通信でArduinoに転送
  - reference/zeuscar-project/pc_control/ の機能をROS2 Jazzyで実装する目標

**開発ツールのインストール（完了）:**
```bash
sudo apt-get install -y build-essential cmake gcc g++ vim python3-pip
```

**ROS2リポジトリの追加（完了）:**
```bash
# Universeリポジトリの有効化
sudo add-apt-repository universe

# ROS2 GPGキーの追加
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2リポジトリの追加
sudo bash -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list'
```

**ROS2 Jazzy Desktopのインストール（失敗）:**
- `sudo apt-get install -y ros-jazzy-desktop` を実行
- ネットワークエラー（IPv6接続失敗、タイムアウト）で一部パッケージのダウンロードに失敗
- archive.ubuntu.comへの接続が不安定だった

#### 発生したトラブルと解決

| ID | 問題 | 解決 |
|----|------|------|
| HOST-010 | ROS2インストール中にネットワークエラー | 次回リトライ（ネットワーク接続は回復済み） |

### 現在の状態
- Ubuntu 24.04インストール完了
- SSH接続安定（Wi-Fi省電力無効化後）
- スリープ無効化完了
- ROS2環境構築手順のドキュメント化完了
- 開発ツール（build-essential, cmake等）インストール完了
- ROS2リポジトリ追加完了
- **ROS2 Jazzy Desktopは未インストール**（ネットワークエラーで失敗）

### 次回やること
1. **ROS2 Jazzy Desktopのインストールをリトライ**
   ```bash
   sudo apt-get update
   sudo apt-get install -y ros-jazzy-desktop
   ```
2. ROS2開発ツールのインストール
   ```bash
   sudo apt-get install -y ros-dev-tools python3-colcon-common-extensions
   ```
3. ROS2環境設定
   - .bashrcにROS2環境変数を追加
   - rosdep init & update
4. Talker-Listenerテストで動作確認
5. Publisherノードの開発開始

---

## 2026-02-01

### 作業内容

#### ROS2環境構築の完了

前回ネットワークエラーで失敗していたROS2インストールをリトライし、成功した。

**ROS2 Jazzy Desktopのインストール（完了）:**
```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop
```

約700以上のパッケージがインストールされ、正常に完了。

**ROS2開発ツールのインストール（完了）:**
```bash
sudo apt-get install -y ros-dev-tools python3-colcon-common-extensions
```

**ROS2環境設定（完了）:**

1. .bashrcへの環境設定追加:
```bash
# ROS2 Jazzy environment setup
source /opt/ros/jazzy/setup.bash

# Colcon argcomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# ROS2 Domain ID (0-232, default 0)
export ROS_DOMAIN_ID=0
```

2. rosdepの初期化:
```bash
sudo rosdep init
rosdep update
```

**Talker-Listenerテスト（成功）:**
```bash
# ターミナル1: Talkerを起動
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker

# ターミナル2: Listenerを起動
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

テスト結果:
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
[INFO] [listener]: I heard: [Hello World: 2]
[INFO] [talker]: Publishing: 'Hello World: 3'
[INFO] [listener]: I heard: [Hello World: 3]
```

PublisherとSubscriberの通信が正常に動作することを確認。

#### ZeusCarパッケージの開発

**Publisherノードの作成（完了）:**

`src/zeuscar_robot_package/zeuscar_robot_package/publisher.py`を作成:
- キーボードから0-10のコマンド番号を入力
- 対応するArduinoコマンド文字列（FORWARD, BACKWARD等）をROS2トピックにパブリッシュ
- トピック名: `/topic`

**setup.pyの更新（完了）:**
```python
entry_points={
    'console_scripts': [
        'publisher_node = zeuscar_robot_package.publisher:main',
        'subscriber_node = zeuscar_robot_package.subscriber:main',
    ],
},
```

**ビルド時のエラーと解決:**

colcon buildで重複パッケージ名エラーが発生:
```
ERROR:colcon:colcon build: Duplicate package names not supported:
- zeuscar_robot_package:
  - reference/zeuscar-project/pc_control/src/zeuscar_robot_package
  - reference/zeuscar-project/ros2/src/zeuscar_robot_package
  - src/zeuscar_robot_package
```

解決方法: `reference/COLCON_IGNORE`ファイルを作成してreferenceディレクトリを除外

**パッケージのビルド（成功）:**
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zeuscar_robot_package
source install/setup.bash
```

**Publisherノードの動作確認（成功）:**
```bash
ros2 run zeuscar_robot_package publisher_node
```

出力:
```
[INFO] [minimal_publisher]: MinimalPublisher node has been started.
...
Enter command (0-10) or 'exit':
```

#### 発生したトラブルと解決

| ID | 問題 | 解決 |
|----|------|------|
| - | 前回のネットワークエラー | ネットワーク回復後にリトライして成功 |
| HOST-010 | colcon build重複パッケージ名エラー | reference/COLCON_IGNOREファイルで除外 |

### 現在の状態
- ✅ Ubuntu 24.04インストール完了
- ✅ SSH接続安定（Wi-Fi省電力無効化済み）
- ✅ スリープ無効化完了
- ✅ 開発ツール（build-essential, cmake等）インストール完了
- ✅ ROS2リポジトリ追加完了
- ✅ ROS2 Jazzy Desktopインストール完了
- ✅ ROS2開発ツール（ros-dev-tools, colcon）インストール完了
- ✅ ROS2環境設定完了（.bashrc, rosdep）
- ✅ Talker-Listenerテスト成功
- ✅ **ZeusCarパッケージのPublisherノード作成完了**
- ✅ **パッケージビルド成功**
- ✅ **Publisherノード動作確認完了**

**ホストPC側のROS2環境構築とPublisherノードの開発が完了した。**

### 次回やること
1. Raspberry Piとの通信テスト（マルチマシンでのトピック通信確認）
2. Subscriberノードの動作確認（Raspberry Pi側）
3. Arduino連携テスト（シリアル通信でモーター制御）

---

## 作業記録テンプレート

```markdown
## YYYY-MM-DD

### 作業内容

#### タスク名
作業の詳細を記載

#### 発生したトラブルと解決

| ID | 問題 | 解決 |
|----|------|------|
| HOST-NNN | 問題の概要 | 解決方法 |

#### ドキュメント更新
- [x] 更新したドキュメント

### 現在の状態
現在の状態を記載
```

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2026-01-31 | 初版作成（2026-01-30、2026-01-31の作業を記録） |
| 2026-01-31 | ROS2環境構築作業の進捗を追記（開発ツール完了、ROS2リポジトリ完了、インストール失敗） |
| 2026-02-01 | ROS2環境構築完了、ZeusCarパッケージPublisherノード作成・ビルド・動作確認完了 |
