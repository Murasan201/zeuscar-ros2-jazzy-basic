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

（エラー発生時に記載）

---

## 通信エラー

（エラー発生時に記載）

---

## ハードウェアエラー

（エラー発生時に記載）

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
