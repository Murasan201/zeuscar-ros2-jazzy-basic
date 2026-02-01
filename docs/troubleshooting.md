# トラブルシューティング

## 概要
本ドキュメントは、ZeusCar ROS2プロジェクトの開発・セットアップ過程で発生したエラーと解決方法を記録する。

---

## エラー一覧

### カテゴリ別索引
- [環境構築エラー](#環境構築エラー)
- [ビルドエラー](#ビルドエラー)
- [実行時エラー](#実行時エラー)
- [通信エラー](#通信エラー)
- [ハードウェアエラー](#ハードウェアエラー)

---

## 環境構築エラー

### ENV-001: ROS2リポジトリ追加時にファイルが空になる

#### 発生日時
2026-01-28

#### エラー内容
ROS2リポジトリを追加するコマンドを実行したが、`/etc/apt/sources.list.d/ros2.list`ファイルが0バイト（空）になった。

#### 発生状況
以下のコマンドを実行した際に発生:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

確認コマンド:
```bash
ls -la /etc/apt/sources.list.d/ros2.list
# 出力: -rw-r--r-- 1 root root 0 Jan 28 19:12 /etc/apt/sources.list.d/ros2.list
```

#### 原因
コマンド置換（`$(...)`) とパイプ、sudo teeの組み合わせで、シェルの解釈順序の問題が発生した可能性がある。

#### 解決方法
`sudo bash -c`を使用して、全体をroot権限のシェルで実行する:

```bash
sudo bash -c 'echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list'
```

または、変数を事前に展開してから実行:
```bash
ARCH=$(dpkg --print-architecture)
CODENAME=$(. /etc/os-release && echo $UBUNTU_CODENAME)
sudo bash -c "echo 'deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main' > /etc/apt/sources.list.d/ros2.list"
```

#### 参考情報
- アーキテクチャ確認: `dpkg --print-architecture` → arm64
- Ubuntuコードネーム確認: `lsb_release -cs` または `/etc/os-release` → noble

---

---

### ENV-002: dpkgロックエラー（unattended-upgrades）

#### 発生日時
2026-01-28

#### エラー内容
```
E: Could not get lock /var/lib/dpkg/lock-frontend. It is held by process 12182 (unattended-upgr)
E: Unable to acquire the dpkg frontend lock (/var/lib/dpkg/lock-frontend), is another process using it?
```

#### 発生状況
`sudo apt-get install -y ros-jazzy-desktop`を実行した際に発生。
バックグラウンドで`unattended-upgrades`（自動アップデート）プロセスが実行中だった。

#### 原因
Ubuntuの自動アップデート機能（unattended-upgrades）がバックグラウンドでパッケージ管理システムを使用中のため、他のaptコマンドがロックを取得できない。

#### 解決方法

**方法1: プロセスの完了を待つ（推奨）**
```bash
# プロセスの状態を確認
ps aux | grep unattended

# 完了を待ってから再度実行
sudo apt-get install -y ros-jazzy-desktop
```

**方法2: unattended-upgradesの完了を待機**
```bash
# systemdでunattended-upgradesの完了を待つ
sudo systemctl stop unattended-upgrades
sudo apt-get install -y ros-jazzy-desktop
sudo systemctl start unattended-upgrades
```

**方法3: プロセス強制終了とロック解除（最終手段）**
```bash
# 1. unattended-upgradeプロセスのPIDを確認
ps aux | grep unattended | grep -v grep

# 2. プロセスを強制終了（PIDは確認した値に置き換え）
sudo kill <PID>

# 3. ロックファイルを強制削除
sudo rm -f /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock /var/cache/apt/archives/lock

# 4. dpkgの状態を修復
sudo dpkg --configure -a
```

> **Warning**: この方法はパッケージシステムが不整合になるリスクがある。
> 実行後は`sudo apt-get update`と`sudo apt-get -f install`で状態を確認すること。

#### 参考情報
- Ubuntuではデフォルトで自動セキュリティアップデートが有効
- Raspberry Pi起動後しばらくは自動アップデートが走る可能性がある
- 2026-01-28: 方法3を実行してROS2インストールを継続（問題なく動作）

---

## ビルドエラー

### BUILD-001: colconで重複パッケージ名エラー

#### 発生日時
2026-01-29

#### エラー内容
```
ERROR:colcon:colcon build: Duplicate package names not supported:
- zeuscar_robot_package:
  - reference/zeuscar-project/pc_control/src/zeuscar_robot_package
  - reference/zeuscar-project/ros2/src/zeuscar_robot_package
  - src/zeuscar_robot_package
```

#### 発生状況
プロジェクトルートで`colcon build`を実行した際に発生。
`reference/`ディレクトリ内に同名のパッケージが存在するため、colconが重複として検出。

#### 原因
colconはデフォルトでカレントディレクトリ以下の全てのROS2パッケージを検索する。
参照用に配置した`reference/`ディレクトリ内のパッケージも検出され、重複エラーとなった。

#### 解決方法
`--paths`オプションで`src/`ディレクトリのみを対象にする:

```bash
# src/ディレクトリ内のパッケージのみをビルド
colcon build --paths src/*

# 特定のパッケージのみをビルドする場合
colcon build --paths src/* --packages-select zeuscar_robot_package
```

または、`COLCON_IGNORE`ファイルを使用して特定ディレクトリを除外:
```bash
# reference/ディレクトリをcolconの検索対象から除外
touch reference/COLCON_IGNORE
```

#### 参考情報
- colconはパッケージ検索時に`COLCON_IGNORE`ファイルがあるディレクトリをスキップする
- 本プロジェクトでは`--paths src/*`を使用する方法を採用

---

---

## 実行時エラー

### RUN-001: シリアルポートのアクセス権エラー（Permission denied）

#### 発生日時
2026-02-01

#### エラー内容
```
[ERROR] [minimal_subscriber]: Failed to open serial port: [Errno 13] could not open port /dev/ttyACM0: [Errno 13] Permission denied: '/dev/ttyACM0'
```

#### 発生状況
ROS2のSubscriberノードを起動してArduinoとシリアル通信しようとした際に発生。
```bash
ros2 run zeuscar_robot_package subscriber_node
```

#### 原因
ユーザーが`dialout`グループに所属していないため、シリアルポート（/dev/ttyACM0）へのアクセス権がない。

#### 解決方法

**方法1: dialoutグループに追加（推奨）**
```bash
# dialoutグループに追加
sudo usermod -a -G dialout $USER

# 反映するには再ログインが必要
# または、現在のセッションで一時的に反映
newgrp dialout
```

**方法2: デバイスファイルのパーミッションを変更（一時的）**
```bash
# 一時的にアクセス権を付与（再起動で元に戻る）
sudo chmod 666 /dev/ttyACM0
```

#### グループ確認方法
```bash
# 現在所属しているグループを確認
groups $USER

# dialoutが含まれていれば問題なし
# 例: pi4 : pi4 adm dialout cdrom sudo ...
```

#### 参考情報
- Linuxではシリアルデバイスへのアクセスにdialoutグループへの所属が必要
- 再ログインせずに反映するには`newgrp dialout`を使用

---

## 通信エラー

（エラー発生時に記載）

---

## ハードウェアエラー

### HW-001: BIOS/LEGACY BOOT OF UEFI-ONLY MEDIA エラー

#### 発生日時
2026-01-30

#### エラー内容
```
ERROR: BIOS/LEGACY BOOT OF UEFI-ONLY MEDIA

This drive was created by Rufus [https://rufus.ie].
It can boot in UEFI mode only but you are trying to
boot it in BIOS/Legacy mode. THIS WILL NOT WORK!
```

#### 発生状況
Intel NUCでRufusで作成したUbuntu 24.04インストールUSB（SSD）から起動しようとした際に発生。
ブートメニュー（F10）からUSBデバイスを選択して起動を試みた。

#### 原因
RufusでUSBメディアを作成する際に以下の設定を使用した:
- パーティション構成: GPT
- ターゲットシステム: UEFI（非CSM）

この設定で作成されたUSBはUEFIモード専用だが、Intel NUCがLegacyモードでUSBを起動しようとしたため、互換性エラーが発生した。

#### 解決方法

**方法1: BIOS設定でUEFIモードを有効にする（推奨）**

1. NUCの電源を入れ、すぐに **F2** を連打してBIOS設定に入る
2. **Boot** タブを開く
3. 以下の設定を変更:
   - **UEFI Boot** → **Enabled**
   - **Legacy Boot** → **Disabled**（または優先度を下げる）
   - **Secure Boot** → **Disabled**（Ubuntuインストール用）
4. **F10** を押して保存・再起動
5. 起動時に **F10** でブートメニューを表示
6. **UEFI: [USB SSD名]** を選択（Legacyではなく**UEFI**プレフィックス付きを選ぶ）

**方法2: RufusでLegacy対応USBを再作成**

Rufusで以下の設定でUSBを再作成:
- パーティション構成: **MBR**
- ターゲットシステム: **BIOS（またはUEFI-CSM）**

> **Note**: 方法1を推奨。現代のPCはUEFIモードでインストールする方が望ましい。

#### 参考情報
- Intel NUCのBIOS設定キー: F2
- Intel NUCのブートメニューキー: F10
- ブートメニューに「UEFI: デバイス名」と「デバイス名」の両方が表示される場合、UEFI付きを選択する

---

## エラー記載テンプレート

```markdown
### エラーID: XXXX-NNN

#### 発生日時
YYYY-MM-DD

#### エラー内容
エラーメッセージをここに記載

#### 発生状況
どのような操作・状況で発生したか

#### 原因
判明した原因

#### 解決方法
解決手順を記載

#### 参考情報
関連リンクや参考資料
```

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2026-01-25 | 初版作成（テンプレート） |
| 2026-01-28 | ENV-001: ROS2リポジトリ追加時のファイル空問題を追加 |
| 2026-01-28 | ENV-002: dpkgロックエラー（unattended-upgrades）を追加 |
| 2026-01-29 | BUILD-001: colconで重複パッケージ名エラーを追加 |
| 2026-01-30 | HW-001: BIOS/Legacy Boot of UEFI-only mediaエラーを追加 |
| 2026-02-01 | RUN-001: シリアルポートのアクセス権エラーを追加 |
