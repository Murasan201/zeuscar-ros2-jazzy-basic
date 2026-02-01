# セットアップガイド - ホストPC

## 概要
本ドキュメントは、ZeusCar ROS2プロジェクトのホストPC（操作側）の環境構築手順を記載する。
ホストPCはROS2 Publisherノードを実行し、Raspberry Pi側にコマンドを送信する役割を担う。

## システム構成

```
[Host PC]          -- ROS2 topic -->  [Raspberry Pi]      -- Serial -->  [Arduino]
Publisher                              Subscriber                         Motor Controller
(本ガイドの対象)                        (setup-guide.md)                   (メカナムホイール)
```

## 対象環境

| 項目 | 内容 |
|------|------|
| PC | デスクトップPC / ノートPC |
| ストレージ | SSD（内蔵または外付け） |
| OS | Ubuntu 24.04 LTS (Noble Numbat) |
| ROS2 | Jazzy Jalisco |

### 本プロジェクトで使用するホストPC

以下は本プロジェクトで実際に使用するホストPCのスペックである。

| 項目 | 内容 |
|------|------|
| PC種別 | Intel NUC |
| CPU | Intel Core i7-1255U (12th Gen) |
| メモリ | 8GB |
| 内蔵SSD | WD Blue SN570 1TB NVMe |
| 元OS | Windows 11 + WSL2 (Ubuntu 22.04) |
| インストール方法 | USB SSDをインストールメディアとして使用し、内蔵SSDにUbuntu 24.04をクリーンインストール |

> **Note**: 元々Windows環境で開発を行っていたが、ROS2 Jazzyを使用するためには
> Ubuntu 24.04のネイティブ環境が必要なため、内蔵SSDにUbuntuをインストールすることを選択した。

---

## 1. インストールの準備

### 1.1 必要なもの

| 項目 | 説明 |
|------|------|
| インストール対象PC | SSDを搭載したPC |
| USBメモリまたはUSB SSD | 8GB以上（インストールメディア用） |
| 別のPC | インストールUSBを作成するため |
| インターネット接続 | 有線LAN推奨（インストール中のアップデート用） |

> **Note**: インストールメディアにはUSBメモリの代わりにUSB SSDも使用可能。
> USB SSDは書き込み速度が速く、インストールメディア作成が高速に完了する。
> 本プロジェクトではUSB SSDをインストールメディアとして使用した。

### 1.2 Ubuntu ISOイメージのダウンロード

Ubuntu公式サイトからデスクトップ版のISOイメージをダウンロードする。

- URL: https://ubuntu.com/download/desktop
- ファイル: `ubuntu-24.04.x-desktop-amd64.iso`（約5GB）

> **Note**: サーバー版ではなく**デスクトップ版**を選択する。
> RViz2等のGUIツールを使用するため、デスクトップ環境が必要。

### 1.3 インストールUSBの作成

#### Windows環境の場合（Rufus使用）

1. Rufusをダウンロード: https://rufus.ie/
2. Rufusを起動
3. 設定:
   - デバイス: USBメモリを選択
   - ブートの種類: ダウンロードしたISOファイルを選択
   - パーティション構成: GPT
   - ターゲットシステム: UEFI（非CSM）
   - ファイルシステム: FAT32（Large）
4. 「スタート」をクリックしてUSBを作成

#### Linux環境の場合

```bash
# USBメモリのデバイス名を確認（例: /dev/sdb）
lsblk

# ISOイメージをUSBに書き込み（sdXは実際のデバイス名に置き換え）
sudo dd if=ubuntu-24.04-desktop-amd64.iso of=/dev/sdX bs=4M status=progress
sudo sync
```

> **Warning**: `dd`コマンドは指定したデバイスを完全に上書きする。
> デバイス名を間違えると他のディスクのデータが消失するので注意。

#### macOS環境の場合（balenaEtcher使用）

1. balenaEtcherをダウンロード: https://www.balena.io/etcher/
2. アプリを起動
3. 「Flash from file」でISOファイルを選択
4. 「Select target」でUSBメモリを選択
5. 「Flash!」をクリック

---

## 2. BIOS/UEFI設定

### 2.1 BIOS設定画面への入り方

PCの電源を入れた直後に、メーカーに応じたキーを押してBIOS設定画面に入る。

| メーカー | BIOS設定キー | ブートメニューキー |
|---------|-------------|-------------------|
| **Intel NUC** | **F2** | **F10** |
| Dell | F2 | F12 |
| HP | F10 | F9 または Esc |
| Lenovo | F1 または F2 | F12 |
| ASUS | F2 または Del | F8 または Esc |
| Acer | F2 または Del | F12 |
| 自作PC | Del または F2 | F11 または F12 |

### 2.2 Intel NUC Visual BIOSでの設定（本プロジェクトで使用）

Intel NUCでは**Visual BIOS**というGUIベースの設定画面が表示される。マウスでも操作可能。

#### 2.2.1 Visual BIOSへのアクセス

1. Intel NUCの電源を入れる
2. 起動直後に **F2** キーを連打
3. Visual BIOS画面（グラフィカルな設定画面）が表示される

#### 2.2.2 Boot Configurationの設定

1. 上部メニューから **「Boot」** タブをクリック
2. **「Boot Configuration」** サブタブを選択
3. 以下の設定を確認・変更:

**UEFI Boot セクション:**

| 設定項目 | 推奨値 | 説明 |
|---------|--------|------|
| Fast Boot | OFF | 高速起動を無効化（トラブル防止） |
| Boot USB Devices First | **ON** | USBデバイスを優先起動 |
| Boot Network Devices Last | ON | ネットワークブートを後回し |

**Boot Devices セクション:**

| 設定項目 | 推奨値 | 説明 |
|---------|--------|------|
| USB | **ON（チェック）** | USBブートを有効化 |
| Internal UEFI Shell | OFF | 不要 |
| Network Boot | 任意 | ネットワークブート |

#### 2.2.3 Secure Bootの無効化

1. **「Boot」** タブ内の **「Secure Boot」** サブタブをクリック
2. **「Secure Boot」** を **Disabled** に変更

> **Note**: Secure Bootが有効だとUbuntuインストール時に問題が発生する場合がある。

#### 2.2.4 Boot Priorityの設定

1. **「Boot」** タブ内の **「Boot Priority」** サブタブをクリック
2. **「UEFI: [USB SSD名]」** をドラッグして最上位に移動
   - または矢印ボタンで順序を変更

> **Important**: 「USB SSD名」と「UEFI: USB SSD名」の両方が表示される場合、
> **必ず「UEFI:」が付いている方**を選択する。「UEFI:」なしはLegacyモード。

#### 2.2.5 設定の保存と終了

1. **F10** キーを押す
2. 「Save changes and exit」で **Yes** を選択
3. PCが再起動する

### 2.3 一般的なBIOS設定（Intel NUC以外）

#### セキュアブートの無効化

Ubuntuのインストールにはセキュアブートを無効にすることを推奨。

1. BIOS設定画面で「Security」または「Boot」タブを探す
2. 「Secure Boot」を「Disabled」に変更
3. 設定を保存して終了

> **Note**: 一部のPCではセキュアブート有効のままでもインストール可能だが、
> ドライバの問題が発生する場合があるため、無効化を推奨。

#### ブート順序の変更

USBメモリからブートするために、ブート順序を変更する。

1. 「Boot」タブを開く
2. ブート順序（Boot Priority / Boot Order）を変更
3. USBデバイスを最優先に設定
4. 設定を保存して終了（通常はF10キー）

> **Note**: 一部のPCでは、起動時にF12等を押すことでブートデバイスを
> 一時的に選択できる（ブートメニュー）。

---

## 3. Ubuntuのインストール

### 3.1 USBからの起動

1. インストールUSB（SSD）をPCに接続
2. PCの電源を入れる
3. **F10** キーを連打してブートメニューを表示（Intel NUCの場合）
4. **「UEFI: [USB SSD名]」** を選択してEnter

> **Important**: 「UEFI:」プレフィックスが付いたデバイスを選択すること。
> 「UEFI:」なしを選択するとLegacyモードで起動し、エラーになる場合がある。

### 3.2 GNU GRUBブートメニュー

USBから起動すると、GNU GRUBブートメニューが表示される。

```
                            GNU GRUB  version 2.12

 *Try or Install Ubuntu
  Ubuntu (safe graphics)
  OEM install (for manufacturers)
  Boot from next volume
  UEFI Firmware Settings
```

#### メニュー項目の説明

| 項目 | 説明 |
|------|------|
| **Try or Install Ubuntu** | 通常はこれを選択（推奨） |
| Ubuntu (safe graphics) | グラフィックの問題がある場合に選択 |
| OEM install | メーカー向け。通常は使用しない |
| Boot from next volume | 別のボリュームから起動 |
| UEFI Firmware Settings | BIOS設定画面に戻る |

1. **「Try or Install Ubuntu」** を選択（デフォルトで選択されている）
2. **Enter** キーを押す
3. Ubuntuのライブ環境が起動する

> **Note**: グラフィックカードとの相性問題で画面が表示されない場合は、
> 「Ubuntu (safe graphics)」を選択する。

### 3.3 インストールウィザードの開始

Ubuntu 24.04では新しいFlutterベースのインストーラーが採用されている。
ライブ環境が起動すると、自動的にインストーラーが開始される。

### 3.4 言語の選択（Welcome）

1. 言語リストから **「日本語」** を選択
2. **「Next」** をクリック

> **Note**: この画面で選択した言語がシステム全体の言語設定になる。

### 3.5 アクセシビリティの設定（Accessibility）

Ubuntu 24.04の新機能。視覚・聴覚・操作のサポート機能を設定できる。

1. 必要に応じてアクセシビリティ機能を有効化:
   - Seeing（視覚）: 高コントラスト、大きな文字、スクリーンリーダー等
   - Hearing（聴覚）: 視覚的アラート等
   - Typing（入力）: スクリーンキーボード、スティッキーキー等
   - Pointing & Clicking（ポインティング）: マウス操作の補助等
2. 特に必要なければそのまま **「Next」** をクリック

### 3.6 キーボードレイアウトの選択（Keyboard layout）

1. レイアウトリストから **「Japanese」** を選択
2. 必要に応じて入力テストフィールドで確認
3. **「Next」** をクリック

### 3.7 インターネット接続（Connect to the internet）

1. 有線LANの場合: 自動的に接続される
2. Wi-Fiの場合: ネットワークを選択してパスワードを入力
3. **「Next」** をクリック

> **Note**: インターネット接続があると、インストール中にアップデートをダウンロードできる。

### 3.8 インストール方法の選択（What do you want to do with Ubuntu?）

1. **「Install Ubuntu」** を選択
2. **「Next」** をクリック

> **Note**: 「Try Ubuntu」を選択すると、インストールせずにライブ環境を試用できる。

### 3.9 インストールタイプの選択（How would you like to install Ubuntu?）

| 選択肢 | 説明 |
|--------|------|
| **Interactive installation** | 手動で設定を行う（推奨） |
| Automated installation | YAMLファイルによる自動インストール |

1. **「Interactive installation」** を選択
2. **「Next」** をクリック

### 3.10 アプリケーションの選択（What apps would you like to install to start with?）

| 選択肢 | 内容 |
|--------|------|
| **Default selection** | 基本的なユーティリティのみ |
| **Extended selection** | Webブラウザ、オフィスツール、ユーティリティ等（推奨） |

1. **「Extended selection」** を選択（ROS2開発に便利なツールが含まれる）
2. オプション（インターネット接続時に表示）:
   - [x] **Install third-party software for graphics and Wi-Fi hardware**（サードパーティドライバ）
   - [x] **Download and install support for additional media formats**（メディアコーデック）
3. **「Next」** をクリック

> **Recommendation**: 両方のオプションにチェックを入れることを推奨。
> Wi-Fiやグラフィックドライバ、動画再生に必要なコーデックがインストールされる。

### 3.11 ディスク設定（How do you want to install Ubuntu?）

#### SSD全体を使用する場合（推奨）

1. **「Erase disk and install Ubuntu」** を選択
2. インストール先のSSD（NVMe等）を選択
3. **「Next」** をクリック

> **Warning**: この操作でSSD上の全データが削除される。
> 必要なデータは事前にバックアップすること。

#### デュアルブートの場合（Windowsと共存）

1. **「Install Ubuntu alongside [Windows]」** を選択
2. スライダーでパーティションサイズを調整
3. Ubuntuには最低50GB以上を割り当てることを推奨
4. **「Next」** をクリック

#### 手動パーティション設定の場合（上級者向け）

1. **「Manual installation」** を選択
2. パーティションテーブルを作成（必要な場合）
3. 以下のパーティションを作成:
   - EFIシステムパーティション: 512MB（/boot/efi、FAT32）
   - ルートパーティション: 残り全て（/、ext4）
   - スワップ: RAMの1〜2倍（オプション）
4. **「Next」** をクリック

### 3.12 ユーザーアカウントの作成（Create your account）

以下の情報を入力する。各項目の役割を理解して適切な値を設定すること。

#### 各項目の詳細説明

| 項目 | 役割 | 表示される場所 |
|------|------|---------------|
| **Your name** | 表示名（フルネーム） | ログイン画面、システム設定 |
| **Your computer's name** | ホスト名 | ネットワーク上の識別名、ターミナルのプロンプト |
| **Pick a username** | ログインID | ターミナル、ファイルパス（/home/username/） |
| **Choose a password** | ログインパスワード | ログイン時、sudo実行時 |

#### Your name（表示名）

ログイン画面やシステム設定に表示される名前。日本語も使用可能。
実名でも任意の名前でもよい。

```
例: ZeusCar User、Taro Yamada、開発者
```

#### Your computer's name（ホスト名）

ネットワーク上でPCを識別するための名前。
ROS2でノード間通信を行う際にも使用される。

- **英小文字、数字、ハイフン（-）のみ使用可能**
- スペースや日本語は使用不可
- 他のPCと重複しない名前にする

```
例: zeuscar-host、ubuntu-nuc、dev-pc
```

#### Pick a username（ユーザー名）

ターミナルでのログインIDおよびホームディレクトリのパスに使用される。

- **英小文字と数字のみ推奨**（先頭は英字）
- 短くて覚えやすい名前がおすすめ
- このユーザー名は変更が難しいため、慎重に決める

```
例: zeuscar、user、pi、dev
```

ターミナルでは以下のように表示される:
```bash
username@hostname:~$

# 例: ユーザー名が zeuscar、ホスト名が zeuscar-host の場合
zeuscar@zeuscar-host:~$
```

#### 本プロジェクトでの推奨設定

| 項目 | 推奨値 | 理由 |
|------|--------|------|
| Your name | `ZeusCar` | プロジェクト名で統一 |
| Your computer's name | `zeuscar-host` | ホストPC（操作側）であることを明示 |
| Pick a username | `zeuscar` | 短くて分かりやすい |
| Choose a password | （任意） | 強力なパスワードを設定 |

> **Note**: Raspberry Pi側のユーザー名も `zeuscar` に統一すると、
> SSH接続時などに分かりやすくなる。

#### パスワードの設定

- 8文字以上を推奨
- 英大文字、英小文字、数字、記号を組み合わせると強力
- 忘れないようにメモしておく（ただし安全な場所に保管）

オプション:
- [x] **Require my password to log in**（推奨: チェックを入れる）
  - セキュリティのため、ログイン時にパスワードを要求する
- [ ] Use Active Directory（通常は不要）
  - 企業のディレクトリサービスを使用する場合のみ

**「Next」** をクリック

### 3.13 タイムゾーンの選択（Select your timezone）

1. 地図上で **「Tokyo」** をクリック、または検索ボックスに「Tokyo」と入力
2. タイムゾーンが **「Asia/Tokyo」** になっていることを確認
3. **「Next」** をクリック

### 3.14 設定の確認（Review your choices）

インストール前に設定内容を確認する画面が表示される。

確認項目:
- Language（言語）: 日本語
- Keyboard layout（キーボード）: Japanese
- Location（場所）: Asia/Tokyo
- Installation type（インストールタイプ）: Erase disk and install Ubuntu
- Disk（ディスク）: [選択したSSD]

1. 設定内容を確認
2. 変更が必要な場合は **「Back」** で戻る
3. 問題なければ **「Install」** をクリック

### 3.15 インストールの実行

1. インストールが開始される
2. 進行状況がプログレスバーで表示される
3. インストール完了まで待機（10〜30分程度）

> **Note**: インストール中にUbuntuの新機能紹介スライドが表示される。

### 3.16 インストール完了

1. 「Installation complete」と表示される
2. **「Restart now」** をクリック
3. 「Please remove the installation medium, then press ENTER」と表示されたら:
   - USB SSDを取り外す
   - **Enter** キーを押す
4. PCが再起動する

---

## 4. 初回起動と初期設定

### 4.1 初回起動

1. PCが再起動し、GRUBブートローダーが表示される
2. 「Ubuntu」を選択（または自動的に起動）
3. ログイン画面が表示されたら、設定したパスワードでログイン

### 4.2 初期設定ウィザード

初回ログイン時に設定ウィザードが表示される場合がある:

1. オンラインアカウント: スキップ可能
2. Livepatch: スキップ可能（Ubuntuアカウントが必要）
3. ヘルプの改善: 任意
4. プライバシー: 任意
5. 「完了」をクリック

### 4.3 システムアップデート

ターミナルを開いて（Ctrl+Alt+T）、システムを最新の状態に更新する。

```bash
# パッケージリストの更新
sudo apt-get update

# パッケージのアップグレード
sudo apt-get upgrade -y

# 再起動（カーネルが更新された場合）
sudo reboot
```

### 4.4 インストール確認

```bash
# OS情報の確認
cat /etc/os-release

# 期待される出力
# PRETTY_NAME="Ubuntu 24.04 LTS"
# VERSION="24.04 LTS (Noble Numbat)"
```

```bash
# カーネル情報の確認
uname -a

# 期待される出力例
# Linux zeuscar-host 6.8.0-xx-generic ... x86_64 GNU/Linux
```

```bash
# ディスク情報の確認（SSDが認識されているか）
lsblk

# 期待される出力例
# NAME   MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
# sda      8:0    0 500.0G  0 disk
# ├─sda1   8:1    0   512M  0 part /boot/efi
# └─sda2   8:2    0 499.5G  0 part /
```

---

## 5. 追加設定（任意）

### 5.1 日本語入力の設定

```bash
# Fcitx5とMozcのインストール
sudo apt-get install -y fcitx5-mozc

# 入力メソッドの設定
im-config -n fcitx5
```

再起動後、システム設定から入力メソッドを設定。

### 5.2 SSHサーバーのインストール

リモートから操作するためにSSHサーバーを設定する。
ROS2開発ではRaspberry Piとの連携やリモートデバッグに必須。

#### 5.2.1 ターミナルを開く

**Ctrl + Alt + T** を押してターミナルを起動する。

#### 5.2.2 パッケージリストの更新

```bash
sudo apt update
```

#### 5.2.3 OpenSSHサーバーのインストール

```bash
sudo apt install -y openssh-server
```

#### 5.2.4 SSHサービスの状態確認

```bash
sudo systemctl status ssh
```

以下のように **active (running)** と表示されれば正常に動作している:

```
● ssh.service - OpenBSD Secure Shell server
     Loaded: loaded (/usr/lib/systemd/system/ssh.service; enabled; preset: enabled)
     Active: active (running) since ...
```

> **Note**: `q` キーを押すと表示を終了できる。

#### 5.2.5 SSHサービスの自動起動を有効化

```bash
sudo systemctl enable ssh
```

これにより、PC起動時に自動的にSSHサーバーが開始される。

#### 5.2.6 ファイアウォールの設定（必要な場合）

Ubuntu 24.04ではデフォルトでファイアウォール（ufw）が無効になっている場合がある。
ファイアウォールを有効にする場合は、SSHを許可する:

```bash
# ファイアウォールの状態確認
sudo ufw status

# ファイアウォールが有効な場合、SSHを許可
sudo ufw allow ssh

# または、ポート番号で指定
sudo ufw allow 22/tcp
```

#### 5.2.7 IPアドレスの確認

SSH接続に必要なIPアドレスを確認する:

```bash
ip a
```

出力例:
```
2: enp0s31f6: <BROADCAST,MULTICAST,UP,LOWER_UP> ...
    inet 192.168.1.100/24 brd 192.168.1.255 scope global dynamic enp0s31f6
```

`inet` の後の **192.168.x.x** がIPアドレス。これをメモしておく。

または、より簡単に確認:
```bash
hostname -I
```

#### 5.2.8 SSH接続テスト（別のPCから）

別のPC（Windows、Mac、Linux）から接続テストを行う:

```bash
ssh ユーザー名@IPアドレス

# 例: ユーザー名が zeuscar、IPアドレスが 192.168.1.100 の場合
ssh zeuscar@192.168.1.100
```

初回接続時は以下のメッセージが表示される:
```
The authenticity of host '192.168.1.100 (192.168.1.100)' can't be established.
ED25519 key fingerprint is SHA256:xxxxx...
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

`yes` と入力してEnter。その後、パスワードを入力してログイン。

#### 5.2.9 SSH設定の確認

```bash
# SSHサーバーの設定ファイル
cat /etc/ssh/sshd_config
```

主な設定項目:

| 設定項目 | デフォルト値 | 説明 |
|---------|-------------|------|
| Port | 22 | SSHポート番号 |
| PermitRootLogin | prohibit-password | rootログインの許可 |
| PasswordAuthentication | yes | パスワード認証の許可 |

> **Note**: セキュリティを強化する場合は、公開鍵認証の設定を推奨。

### 5.3 固定IPアドレスの設定（任意）

ROS2のネットワーク通信を安定させるために、固定IPを設定することを推奨。

1. 「設定」→「ネットワーク」を開く
2. 接続中のネットワークの歯車アイコンをクリック
3. 「IPv4」タブを選択
4. 「手動」を選択し、以下を設定:
   - アドレス: 192.168.x.x（ネットワーク環境に合わせる）
   - ネットマスク: 255.255.255.0
   - ゲートウェイ: 192.168.x.1
   - DNS: 8.8.8.8, 8.8.4.4
5. 「適用」をクリック

### 5.4 スリープの無効化（推奨）

開発用PCではスリープを無効化することを強く推奨する。
スリープに入るとSSH接続が切断され、リモート作業が中断されてしまう。

> **Important**: SSH接続中であってもスリープは発生する。
> SSH接続はスリープを防ぐ要因としては扱われない。

#### GUI設定

1. **設定** → **電源** を開く
2. 以下を設定:

| 項目 | 推奨設定 |
|------|---------|
| 画面のブランク | しない |
| 自動サスペンド | オフ |
| 電源ボタンの動作 | 電源オフ（任意） |

#### コマンドで設定（推奨）

GUIだけでなく、コマンドでも設定することで確実に無効化できる。

##### 1. システムスリープの無効化

```bash
# スリープ・サスペンド・ハイバネートを無効化
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

> **Note**: 「does not exist, proceeding anyway」と表示されても問題ない。
> マスク処理は正常に完了している。

設定確認:
```bash
systemctl status sleep.target
```

以下のように「masked」と表示されれば無効化されている:
```
○ sleep.target
     Loaded: masked (Reason: Unit sleep.target is masked.)
     Active: inactive (dead)
```

##### 2. 画面ブランク（スクリーンセーバー）の無効化

GNOMEデスクトップの画面ブランク設定をコマンドで無効化する:

```bash
# アイドル時の画面オフを無効化（0 = 無効）
gsettings set org.gnome.desktop.session idle-delay 0
```

設定確認:
```bash
gsettings get org.gnome.desktop.session idle-delay
```

`uint32 0` と表示されれば無効化されている。

##### 3. 電源管理の無効化

AC電源接続時のスリープを無効化する:

```bash
# AC電源時の自動スリープを無効化
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'

# AC電源時のスリープまでの時間を0に（念のため）
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0
```

設定確認:
```bash
gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type
gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout
```

それぞれ `'nothing'` と `0` と表示されれば設定完了。

##### 4. ロック画面の無効化（任意）

SSH接続のみで使用する場合、ロック画面も無効化すると便利:

```bash
# 自動ロックを無効化
gsettings set org.gnome.desktop.screensaver lock-enabled false

# アイドル時のロックを無効化
gsettings set org.gnome.desktop.screensaver idle-activation-enabled false
```

##### まとめ: 全設定を一括実行

以下のコマンドを順番に実行すれば、すべての省電力設定を無効化できる:

```bash
# システムスリープの無効化
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# 画面ブランクの無効化
gsettings set org.gnome.desktop.session idle-delay 0

# 電源管理の無効化
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0

# ロック画面の無効化（任意）
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.desktop.screensaver idle-activation-enabled false
```

#### 元に戻す場合

```bash
# システムスリープを元に戻す
sudo systemctl unmask sleep.target suspend.target hibernate.target hybrid-sleep.target

# 画面ブランクを元に戻す（5分 = 300秒）
gsettings set org.gnome.desktop.session idle-delay 300

# 電源管理を元に戻す
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'suspend'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 1200

# ロック画面を元に戻す
gsettings set org.gnome.desktop.screensaver lock-enabled true
gsettings set org.gnome.desktop.screensaver idle-activation-enabled true
```

### 5.5 Wi-Fi省電力の無効化（Wi-Fi接続の場合は必須）

Wi-Fi接続を使用する場合、**省電力機能を必ず無効化する**こと。
省電力が有効だとSSH接続が断続的に切断され、リモート作業が困難になる。

> **Important**: この問題はスリープ無効化だけでは解決しない。
> Wi-Fiアダプタ自体の省電力機能を個別に無効化する必要がある。

#### 5.5.1 Wi-Fi接続名の確認

```bash
sudo nmcli connection show
```

出力例:
```
NAME                UUID                                  TYPE      DEVICE
Baffalo-5330-WPA3   xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx  wifi      wlan0
```

`NAME` 列の値（例: `Baffalo-5330-WPA3`）をメモする。

#### 5.5.2 省電力を無効化

```bash
sudo nmcli connection modify "接続名" 802-11-wireless.powersave 2
```

例:
```bash
sudo nmcli connection modify "Baffalo-5330-WPA3" 802-11-wireless.powersave 2
```

#### 5.5.3 接続を再起動して設定を反映

```bash
sudo nmcli connection down "接続名" && sudo nmcli connection up "接続名"
```

例:
```bash
sudo nmcli connection down "Baffalo-5330-WPA3" && sudo nmcli connection up "Baffalo-5330-WPA3"
```

#### 5.5.4 設定の確認

```bash
nmcli connection show "接続名" | grep powersave
```

出力例:
```
802-11-wireless.powersave:              2 (disable)
```

`2 (disable)` と表示されれば設定完了。

#### powersave値の意味

| 値 | 意味 |
|----|------|
| 0 | デフォルト（プロファイルに従う） |
| 1 | 無視（省電力設定を無視） |
| **2** | **無効（省電力を完全に無効化）← 推奨** |
| 3 | 有効（省電力を有効化） |

> **Recommendation**: 開発用PCでは有線LAN接続を推奨。
> 有線LANでは省電力による切断問題が発生しない。

---

## 6. 開発環境のセットアップ

Ubuntuインストール後、ROS2開発に必要な基本ツールをインストールする。

### 6.1 必要なシステムパッケージのインストール

```bash
# パッケージリストの更新
sudo apt-get update

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

### 6.2 Gitのセットアップ

#### 6.2.1 Gitのインストール確認

```bash
# Gitバージョンの確認
git --version

# 期待される出力例
# git version 2.43.0
```

#### 6.2.2 Gitユーザー情報の設定

コミット時に使用するユーザー名とメールアドレスを設定する。

```bash
# ユーザー名の設定
git config --global user.name "Your Name"

# メールアドレスの設定
git config --global user.email "your.email@example.com"

# 設定の確認
git config --global --list
```

#### 6.2.3 デフォルトブランチ名の設定（任意）

```bash
# デフォルトブランチ名をmainに設定
git config --global init.defaultBranch main
```

### 6.3 Python開発環境の確認

#### 6.3.1 Pythonバージョンの確認

```bash
# Python3バージョンの確認
python3 --version

# 期待される出力例
# Python 3.12.3
```

#### 6.3.2 pipバージョンの確認

```bash
# pip3バージョンの確認
pip3 --version

# 期待される出力例
# pip 24.0 from /usr/lib/python3/dist-packages/pip (python 3.12)
```

#### 6.3.3 Python開発パッケージの確認

```bash
# python3-devパッケージの確認
dpkg -l | grep python3-dev

# インストールされていない場合はインストール
sudo apt-get install -y python3-dev
```

### 6.4 C++開発環境の確認

#### 6.4.1 GCCバージョンの確認

```bash
# gccバージョンの確認
gcc --version

# 期待される出力例
# gcc (Ubuntu 13.2.0-23ubuntu4) 13.2.0
```

#### 6.4.2 G++バージョンの確認

```bash
# g++バージョンの確認
g++ --version

# 期待される出力例
# g++ (Ubuntu 13.2.0-23ubuntu4) 13.2.0
```

#### 6.4.3 CMakeバージョンの確認

```bash
# cmakeバージョンの確認
cmake --version

# 期待される出力例
# cmake version 3.28.3
```

---

## 7. ROS2 Jazzyのインストール

ROS2 Jazzy JaliscoはUbuntu 24.04 LTS向けのROS2ディストリビューションである。

公式ドキュメント: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 7.1 ロケール設定

ROS2はUTF-8ロケールを必要とする。

```bash
# 現在のロケール確認
locale
```

> **Note**: `LANG=C.UTF-8`または`LANG=en_US.UTF-8`のように、UTF-8が含まれていれば問題ない。

UTF-8ロケールが設定されていない場合のみ以下を実行:

```bash
sudo apt-get update && sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 設定確認
locale
```

### 7.2 リポジトリの追加

ROS2パッケージをインストールするために、ROS2のaptリポジトリを追加する。

#### 7.2.1 Universeリポジトリの有効化

```bash
# Universeリポジトリを有効化
sudo add-apt-repository universe
```

#### 7.2.2 ROS2 GPGキーの追加

```bash
# 必要なツールのインストール
sudo apt-get update && sudo apt-get install -y curl

# ROS2 GPGキーをダウンロードして追加
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

#### 7.2.3 ROS2リポジトリの追加

```bash
# リポジトリをソースリストに追加
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

> **Note**: ホストPC（x86_64）では `arch=amd64` が自動的に設定される。
> Raspberry Pi（ARM64）では `arch=arm64` となる。

リポジトリが正しく追加されたことを確認:

```bash
cat /etc/apt/sources.list.d/ros2.list

# 期待される出力（ホストPCの場合）
# deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main
```

#### 7.2.4 パッケージリストの更新

```bash
# aptパッケージリストを更新
sudo apt-get update

# システムパッケージを最新に（ROS2インストール前に推奨）
sudo apt-get upgrade -y
```

### 7.3 ROS2パッケージのインストール

#### 7.3.1 ROS2 Desktopのインストール

Desktop版には、ROS2コアに加えて可視化ツール（RViz2）やデモパッケージが含まれる。

```bash
# ROS2 Jazzy Desktop版のインストール
sudo apt-get install -y ros-jazzy-desktop
```

> **Note**: インストールには時間がかかる（数分〜十数分程度）。
> 依存パッケージも含めて約2GB程度のダウンロードとなる。

> **Troubleshooting**: `Could not get lock /var/lib/dpkg/lock-frontend`エラーが発生した場合は、
> `unattended-upgrades`（自動セキュリティアップデート）が実行中の可能性がある。
> 完了を待つか、`sudo systemctl stop unattended-upgrades`で一時停止する。
> 詳細は[トラブルシューティング HOST-009](troubleshooting-host-pc.md#host-009-aptdpkgのロックエラーunattended-upgrades)を参照。

#### 7.3.2 開発ツールのインストール

ROS2パッケージのビルドに必要な開発ツールをインストールする。

```bash
# ROS2開発ツールのインストール
sudo apt-get install -y ros-dev-tools
```

> **Note**: `ros-dev-tools`には以下のツールが含まれる:
> - `rosdep`: ROS2パッケージの依存関係を自動解決
> - `vcstool`: バージョン管理システムツール
> - `colcon`: ROS2ビルドツール
> - その他のROS2開発用ユーティリティ

### 7.4 環境設定

#### 7.4.1 ROS2環境のセットアップスクリプト

ROS2を使用するには、セットアップスクリプトをsourceする必要がある。

```bash
# 手動でROS2環境をセットアップ（現在のターミナルのみ）
source /opt/ros/jazzy/setup.bash
```

#### 7.4.2 自動読み込み設定

毎回手動でsourceするのは手間なので、.bashrcに追記して自動化する。

```bash
# .bashrcにROS2セットアップを追加
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 設定を反映
source ~/.bashrc
```

#### 7.4.3 環境変数の確認

```bash
# ROS2関連の環境変数を確認
printenv | grep -i ROS

# 期待される出力例
# ROS_VERSION=2
# ROS_PYTHON_VERSION=3
# ROS_DISTRO=jazzy
```

### 7.5 colconのインストール

colconはROS2のビルドツールである。

```bash
# colcon関連パッケージのインストール
sudo apt-get install -y python3-colcon-common-extensions
```

#### 7.5.1 colcon補完の設定

colconコマンドのタブ補完を有効にする。

```bash
# colcon補完を.bashrcに追加
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# 設定を反映
source ~/.bashrc
```

### 7.6 rosdepのセットアップ

rosdepはROS2パッケージの依存関係を自動解決するツールである。

#### 7.6.1 rosdepの初期化

```bash
# rosdepの初期化（システム全体で一度だけ実行）
sudo rosdep init
```

> **Note**: 既に初期化済みの場合は「already initialized」と表示される。エラーではない。

#### 7.6.2 rosdepデータベースの更新

```bash
# rosdepデータベースの更新（ユーザーごとに実行）
rosdep update
```

### 7.7 ROS2動作確認

ROS2が正しくインストールされたことを確認する。

#### 7.7.1 バージョン確認

```bash
# ROS2のバージョン確認
ros2 --version

# 期待される出力例
# ros2 0.32.0
```

#### 7.7.2 Talker-Listenerテスト

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

**Ctrl+C** で各ノードを終了する。

#### 7.7.3 トピック一覧の確認

```bash
# 現在のトピック一覧を表示
ros2 topic list

# 期待される出力例（ノードが起動していない場合）
# /parameter_events
# /rosout
```

---

## 8. ROS2ワークスペースのセットアップ

### 8.1 追加の開発パッケージ

プロジェクトで必要となる追加パッケージをインストールする。

```bash
# シリアル通信用（Arduino連携）
sudo apt-get install -y python3-serial

# その他の開発ツール
sudo apt-get install -y python3-pytest python3-pytest-cov
```

### 8.2 ワークスペースの作成

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

#### 8.2.1 ワークスペース環境の自動読み込み

```bash
# ワークスペースのセットアップを.bashrcに追加
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 設定を反映
source ~/.bashrc
```

### 8.3 プロジェクトのクローン（任意）

GitHubからプロジェクトをクローンする場合:

```bash
# srcディレクトリに移動
cd ~/ros2_ws/src

# プロジェクトをクローン
git clone https://github.com/your-username/zeuscar-ros2-jazzy-basic.git

# ワークスペースに戻ってビルド
cd ~/ros2_ws
colcon build --paths src/*

# 環境を更新
source install/setup.bash
```

---

## 9. ROS2ネットワーク設定（マルチマシン構成）

ホストPCとRaspberry Pi間でROS2トピックを通信するための設定。

### 9.1 ROS2のネットワーク通信方式

ROS2はDDS（Data Distribution Service）を使用してノード間通信を行う。
デフォルトでは**同一ネットワーク内のノードは自動的に発見**される。

### 9.2 前提条件

| 要件 | 説明 |
|------|------|
| 同一ネットワーク | ホストPCとRaspberry Piが同じLAN内にあること |
| ファイアウォール | ROS2通信ポート（UDP 7400-7500等）が開いていること |
| ROS_DOMAIN_ID | 両マシンで同じ値を設定（デフォルトは0） |

### 9.3 ROS_DOMAIN_IDの設定

同じネットワーク内に複数のROS2システムがある場合、DOMAIN_IDで分離できる。

```bash
# DOMAIN_IDを設定（0〜232の範囲）
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc

# 確認
echo $ROS_DOMAIN_ID
```

> **Important**: ホストPCとRaspberry Piで**同じROS_DOMAIN_ID**を設定すること。

### 9.4 通信確認

**ホストPC（Publisher側）:**
```bash
# テスト用にトピックをPublish
ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello from Host PC'"
```

**Raspberry Pi（Subscriber側）:**
```bash
# トピックをSubscribe
ros2 topic echo /test_topic
```

Raspberry Pi側で「Hello from Host PC」が表示されれば通信成功。

### 9.5 トラブルシューティング: 通信できない場合

#### 9.5.1 ファイアウォールの確認

```bash
# ファイアウォールの状態確認
sudo ufw status

# ROS2用ポートを開放（必要な場合）
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

#### 9.5.2 ネットワークインターフェースの確認

```bash
# IPアドレスの確認
hostname -I

# 両マシンから互いにpingで疎通確認
ping 192.168.x.x
```

#### 9.5.3 DDS設定の確認

デフォルトのDDS（FastDDS）でマルチキャストが使えない場合、ユニキャストに変更:

```bash
# FastDDS設定ファイルの作成
mkdir -p ~/fastdds
cat << 'EOF' > ~/fastdds/fastdds.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <metatrafficUnicastLocatorList>
                    <locator/>
                </metatrafficUnicastLocatorList>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>192.168.x.x</address>  <!-- 相手のIPアドレス -->
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

# 環境変数で設定ファイルを指定
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds/fastdds.xml" >> ~/.bashrc
source ~/.bashrc
```

---

## 10. ZeusCarパッケージの開発

本プロジェクトでは、プロジェクトルートをROS2ワークスペースとして使用する。

### 10.1 プロジェクト構造

```
zeuscar-ros2-jazzy-basic/         # ROS2ワークスペース
├── src/                          # パッケージ格納ディレクトリ
│   └── zeuscar_robot_package/    # ZeusCar制御パッケージ
├── build/                        # ビルド成果物（自動生成）
├── install/                      # インストール先（自動生成）
├── log/                          # ログ（自動生成）
└── reference/                    # 参照用（COLCON_IGNOREで除外）
```

### 10.2 COLCON_IGNOREの設定

referenceディレクトリには参照用の同名パッケージが含まれるため、colconから除外する。

```bash
# referenceディレクトリを除外
touch reference/COLCON_IGNORE
```

> **Note**: `COLCON_IGNORE`ファイルが存在するディレクトリは、colconのビルド対象から除外される。

### 10.3 Publisherノードの作成

#### 10.3.1 publisher.pyの作成

`src/zeuscar_robot_package/zeuscar_robot_package/publisher.py`を作成:

```python
"""
ZeusCar Publisher Node
キーボードから入力されたコマンド番号（0-10）をROS2トピックにパブリッシュする
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# コマンド番号とArduinoコマンド文字列のマッピング
COMMANDS_MAP = {
    0:  "FORWARD",       # 前進
    1:  "BACKWARD",      # 後退
    2:  "LEFT",          # 左旋回
    3:  "RIGHT",         # 右旋回
    4:  "LEFTFORWARD",   # 左前進
    5:  "RIGHTFORWARD",  # 右前進
    6:  "LEFTBACKWARD",  # 左後退
    7:  "RIGHTBACKWARD", # 右後退
    8:  "TURNLEFT",      # 左回転
    9:  "TURNRIGHT",     # 右回転
    10: "STOP"           # 停止
}

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.get_logger().info('MinimalPublisher node has been started.')

    def publish_command(self, command_str: str):
        msg = String()
        msg.data = command_str
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: "{command_str}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(minimal_publisher, timeout_sec=0.1)
            user_input = input("Enter command number (0=FORWARD, 1=BACKWARD, etc.) or 'exit': ").strip()
            if user_input.lower() == 'exit':
                print("Exiting command input loop.")
                break
            if user_input.isdigit():
                cmd_num = int(user_input)
                if cmd_num in COMMANDS_MAP:
                    minimal_publisher.publish_command(COMMANDS_MAP[cmd_num])
                else:
                    print("Invalid number. Please enter a value between 0 and 10.")
            else:
                print("Invalid input. Please enter a number (0-10) or 'exit'.")
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 10.3.2 setup.pyへのエントリポイント追加

`src/zeuscar_robot_package/setup.py`の`entry_points`に`publisher_node`を追加:

```python
entry_points={
    'console_scripts': [
        'publisher_node = zeuscar_robot_package.publisher:main',
        'subscriber_node = zeuscar_robot_package.subscriber:main',
    ],
},
```

### 10.4 パッケージのビルド

```bash
# ROS2環境をソース
source /opt/ros/jazzy/setup.bash

# パッケージをビルド
colcon build --packages-select zeuscar_robot_package

# ビルド成果物をソース
source install/setup.bash
```

期待される出力:
```
Starting >>> zeuscar_robot_package
Finished <<< zeuscar_robot_package [X.XXs]

Summary: 1 package finished [X.XXs]
```

### 10.5 Publisherノードの実行

```bash
# ROS2環境とワークスペースをソース
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Publisherノードを実行
ros2 run zeuscar_robot_package publisher_node
```

出力例:
```
[INFO] [minimal_publisher]: MinimalPublisher node has been started.
Enter command number (0=FORWARD, 1=BACKWARD, etc.) or 'exit':
```

コマンドを入力すると、`/topic`トピックにメッセージがパブリッシュされる。

### 10.6 動作確認

別のターミナルでトピックを監視:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /topic
```

Publisherノードでコマンドを入力すると、echoに表示される:
```
data: 'FORWARD'
---
data: 'STOP'
---
```

---

## 11. 次のステップ

ZeusCarパッケージの開発が完了したら、以下に進む:

1. **Raspberry Piとの通信テスト** - マルチマシンでのトピック通信確認
2. **Subscriberノードの動作確認** - Raspberry Pi側でコマンドを受信
3. **Arduino連携** - シリアル通信でモーター制御

---

## トラブルシューティング

詳細なトラブルシューティングは [troubleshooting-host-pc.md](troubleshooting-host-pc.md) を参照。

### BIOS/LEGACY BOOT OF UEFI-ONLY MEDIA エラー

RufusでUEFI専用（GPT）で作成したUSBを、Legacyモードで起動しようとした場合に発生。

**解決方法:**
1. ブートメニュー（F10）で **「UEFI: USB名」** を選択（「UEFI:」付きを選ぶ）
2. または、BIOS設定で「Boot USB Devices First」を **ON** にする
3. BIOS設定で「Legacy Boot」を **Disabled** にする

### USBからブートできない

- BIOS設定でセキュアブートが無効になっているか確認
- ブート順序でUSBが最優先になっているか確認
- USBメモリが正しく作成されているか確認（別のツールで再作成を試す）
- **ブートメニューで「UEFI:」付きのデバイスを選択しているか確認**

### インストール中にフリーズする

- グラフィックドライバの問題の可能性
- GRUBメニューで **「Ubuntu (safe graphics)」** を選択して起動を試す
- または「Try or Install Ubuntu」で「e」キーを押し、`quiet splash`を`nomodeset`に変更して起動

### SSDが認識されない

- BIOS設定でSATAモードがAHCIになっているか確認
- SSDが正しく接続されているか確認

---

## 6. ROS2通信テスト

ホストPCとRaspberry Pi間でROS2通信が正常に動作するかテストする。

### 6.1 前提条件

- ホストPCとRaspberry Piが同一ネットワークに接続されていること
- 両方でROS2 Jazzyがインストール済みであること
- Raspberry PiにArduinoが接続されていること（/dev/ttyACM0）

### 6.2 Raspberry Pi側の準備

Raspberry Piでターミナルを開き、以下を実行:

```bash
# ワークスペースに移動
cd ~/work/zeuscar-ros2-jazzy-basic

# 環境のセットアップ
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Subscriberノードを起動
ros2 run zeuscar_robot_package subscriber_node
```

期待される出力:
```
[INFO] [minimal_subscriber]: Serial connection established on /dev/ttyACM0 at 9600 bps.
```

### 6.3 ホストPC側からの送信

ホストPCでターミナルを開き、Publisherノードを実行:

```bash
# ROS2環境のセットアップ
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash  # ワークスペースのパスは環境に合わせる

# Publisherノードを起動
ros2 run zeuscar_robot_package publisher_node
```

または、`ros2 topic pub`コマンドで直接テスト:

```bash
# FORWARD コマンドを送信
ros2 topic pub --once /topic std_msgs/msg/String "{data: 'FORWARD'}"

# STOP コマンドを送信
ros2 topic pub --once /topic std_msgs/msg/String "{data: 'STOP'}"
```

### 6.4 動作確認

1. **Raspberry Pi側のログを確認**
   ```
   [INFO] [minimal_subscriber]: I heard: 'FORWARD'
   [INFO] [minimal_subscriber]: Sent to Arduino: FORWARD
   ```

2. **Arduinoの動作を確認**
   - モーターが指定した方向に動作すること

### 6.5 コマンド一覧

| 番号 | コマンド | 動作 |
|------|----------|------|
| 0 | FORWARD | 前進 |
| 1 | BACKWARD | 後退 |
| 2 | LEFT | 左移動 |
| 3 | RIGHT | 右移動 |
| 4 | LEFTFORWARD | 左前進 |
| 5 | RIGHTFORWARD | 右前進 |
| 6 | LEFTBACKWARD | 左後退 |
| 7 | RIGHTBACKWARD | 右後退 |
| 8 | TURNLEFT | 左旋回 |
| 9 | TURNRIGHT | 右旋回 |
| 10 | STOP | 停止 |

### 6.6 トラブルシューティング

#### ROS2通信ができない場合

```bash
# ROS_DOMAIN_IDを確認（両方で同じ値にする）
echo $ROS_DOMAIN_ID

# 設定されていない場合は同じ値を設定
export ROS_DOMAIN_ID=0
```

#### トピックが見つからない場合

```bash
# トピック一覧を確認
ros2 topic list

# トピックの詳細を確認
ros2 topic info /topic
```

#### シリアルポートエラーの場合

```bash
# Arduinoのポートを確認
ls -la /dev/ttyACM*

# シリアルポートのアクセス権を確認
sudo usermod -a -G dialout $USER
# 再ログインが必要
```

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2026-02-01 | ROS2通信テスト手順を追加（12節） |
| 2026-02-01 | 10. ZeusCarパッケージの開発セクションを追加 |
| 2026-02-01 | 11. 次のステップセクションを追加 |
| 2026-02-01 | Publisherノード名を参照プロジェクトに合わせてMinimalPublisherに修正 |
| 2026-01-31 | 6. 開発環境のセットアップ（Git, Python, C++）を追加 |
| 2026-01-31 | 7. ROS2 Jazzyのインストール手順を追加 |
| 2026-01-31 | 8. ROS2ワークスペースのセットアップを追加 |
| 2026-01-31 | 9. ROS2ネットワーク設定（マルチマシン構成）を追加 |
| 2026-01-31 | スリープ・画面ブランク・電源管理の詳細設定手順を追加（5.4節） |
| 2026-01-31 | Wi-Fi省電力無効化の設定手順を追加（5.5節） |
| 2026-01-30 | スリープ無効化の設定手順を追加 |
| 2026-01-30 | SSHサーバーのインストール手順を詳細化 |
| 2026-01-30 | Ubuntu 24.04の新インストーラー手順を詳細化（Flutter版インストーラー対応） |
| 2026-01-30 | Intel NUC Visual BIOSの詳細設定手順を追加 |
| 2026-01-30 | GNU GRUBブートメニューの詳細説明を追加 |
| 2026-01-30 | UEFI/Legacyブートエラーのトラブルシューティングを追加 |
| 2026-01-30 | 実際に使用するホストPCのスペック情報を追加、USB SSD使用に関する説明を追加 |
| 2026-01-29 | 初版作成（SSDへのUbuntuインストール手順） |
