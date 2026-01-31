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

## 次のステップ

Ubuntuのインストールが完了したら、以下のセットアップに進む:

1. **開発環境のセットアップ** - Git, Python, C++等
2. **ROS2 Jazzyのインストール** - セットアップガイド（setup-guide.md）のセクション2を参照
3. **Publisherパッケージの作成** - PC側の制御ノード

> **Note**: ROS2のインストール手順はRaspberry Pi用セットアップガイド（setup-guide.md）の
> セクション2「ROS2 Jazzyのインストール」とほぼ同じ手順で実行できる。
> ただし、アーキテクチャがamd64である点のみ異なる。

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

## 変更履歴

| 日付 | 内容 |
|------|------|
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
