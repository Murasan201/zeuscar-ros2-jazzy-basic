# トラブルシューティング - ホストPC

## 概要
本ドキュメントは、ZeusCar ROS2プロジェクトのホストPC（Intel NUC）セットアップ過程で発生したエラーと解決方法を記録する。

---

## エラー一覧

### カテゴリ別索引
- [Ubuntuインストールエラー](#ubuntuインストールエラー)
- [BIOS/UEFI設定エラー](#biosuefi設定エラー)
- [ドライバ・ハードウェアエラー](#ドライバハードウェアエラー)

---

## Ubuntuインストールエラー

### HOST-001: BIOS/LEGACY BOOT OF UEFI-ONLY MEDIA エラー

#### 発生日時
2026-01-30

#### 発生環境
- PC: Intel NUC
- インストールメディア: USB SSD（Rufusで作成）
- インストール対象OS: Ubuntu 24.04 LTS

#### エラー内容
```
ERROR: BIOS/LEGACY BOOT OF UEFI-ONLY MEDIA

This drive was created by Rufus [https://rufus.ie].
It can boot in UEFI mode only but you are trying to
boot it in BIOS/Legacy mode. THIS WILL NOT WORK!

To remove this message you need to do ONE of the following:
o If this computer supports UEFI, go to your UEFI settings
  and lower or disable the priority of CSM/Legacy mode.
o OR Recreate the drive in Rufus and use:
  * Partition scheme -> MBR.
  * Target system -> BIOS (...)
o OR Erase the whole drive in Rufus by selecting:
  * Boot Type -> Non bootable

Note: You may also see this message if you installed a new
OS and your computer is unable to boot that OS in UEFI mode.

Please remove this media and press any key to reboot
```

#### 発生状況
1. RufusでUbuntu 24.04 ISOをUSB SSDに書き込み
   - パーティション構成: GPT
   - ターゲットシステム: UEFI（非CSM）
2. Intel NUCにUSB SSDを接続して起動
3. ブートメニュー（F10）からUSBデバイスを選択
4. 上記エラーが表示されて起動できない

#### 原因
RufusでUSBメディアを**UEFIモード専用（GPT）**で作成したが、Intel NUCが**Legacyモード**でUSBを起動しようとしたため、互換性エラーが発生。

ブートメニューでUSBデバイスを選択する際に、「UEFI:」プレフィックスが付いていないデバイスを選択すると、Legacyモードでの起動を試みる。

#### 解決方法

**方法1: ブートメニューでUEFIデバイスを選択する（最も簡単）**

1. Intel NUCの電源を入れ、**F10** を連打してブートメニューを表示
2. 一覧から **「UEFI: [USB SSD名]」** を選択（「UEFI:」プレフィックス付き）
3. Enterキーで起動

> **Note**: 「USB SSD名」と「UEFI: USB SSD名」の両方が表示される場合がある。
> 必ず「UEFI:」が付いている方を選択する。

**方法2: BIOS設定でUEFIモードを優先にする**

1. Intel NUCの電源を入れ、**F2** を連打してBIOS設定（Visual BIOS）に入る
2. **Boot** タブを開く
3. 以下の設定を変更:

| 設定項目 | 変更後の値 |
|---------|-----------|
| UEFI Boot | Enabled |
| Legacy Boot | Disabled |
| Secure Boot | Disabled |

4. **Boot Priority** でUSB SSD（UEFI）を最優先に設定
5. **F10** または「Save & Exit」で保存して再起動

**方法3: RufusでLegacy対応USBを再作成（非推奨）**

どうしてもUEFIモードで起動できない場合のみ:

1. Rufusを起動
2. 以下の設定でUSBを再作成:
   - パーティション構成: **MBR**
   - ターゲットシステム: **BIOS（またはUEFI-CSM）**

> **Warning**: 現代のPCではUEFIモードでのインストールを推奨。
> Legacyモードは互換性のためだけに残されている古い方式。

#### 参考情報
- Intel NUCのBIOS設定キー: **F2**
- Intel NUCのブートメニューキー: **F10**
- 新しいIntel NUCは**Visual BIOS**（GUI）を採用

---

## BIOS/UEFI設定エラー

### HOST-002: Intel NUC Visual BIOS の操作方法

#### 概要
Intel NUCの新しいモデルでは、従来のテキストベースBIOSではなく、**Visual BIOS**と呼ばれるGUIベースの設定画面が表示される。マウスでも操作可能。

#### Visual BIOSへのアクセス方法

1. Intel NUCの電源を入れる
2. 起動直後に **F2** キーを連打
3. Visual BIOS画面（グラフィカルな設定画面）が表示される

#### 主要な設定項目の場所

| 設定項目 | 場所 |
|---------|------|
| Boot Priority（起動順序） | Boot タブ → Boot Priority |
| UEFI Boot 有効/無効 | Boot タブ → Boot Configuration |
| Legacy Boot 有効/無効 | Boot タブ → Boot Configuration |
| Secure Boot 有効/無効 | Security タブ → Secure Boot |

#### Boot Priorityの変更方法

Visual BIOSでは以下の方法で起動順序を変更できる:

1. **ドラッグ＆ドロップ**: デバイスをマウスでドラッグして順序を変更
2. **矢印ボタン**: デバイスを選択して上下矢印ボタンで移動
3. **キーボード**: 上下キーで選択、+/-キーで順序変更

#### 設定の保存方法

- 画面右下または上部の **「Save & Exit」** ボタンをクリック
- または **F10** キーを押す
- 確認ダイアログで「Yes」を選択

#### Ubuntuインストール用の推奨設定

| 設定項目 | 推奨値 | 理由 |
|---------|--------|------|
| UEFI Boot | Enabled | 現代的なブート方式 |
| Legacy Boot | Disabled | UEFIのみ使用 |
| Secure Boot | Disabled | Ubuntuインストール時の互換性確保 |
| Boot Priority | UEFI: USB を最優先 | USBからインストーラー起動 |

---

## ドライバ・ハードウェアエラー

### HOST-003: インストール完了後に再びインストールメニューが表示される

#### 発生日時
2026-01-30

#### エラー内容
Ubuntuのインストールが完了して「Restart now」をクリックした後、再びGNU GRUBのインストールメニューが表示される。

#### 発生状況
1. Ubuntuのインストールが完了
2. 「Restart now」をクリック
3. 再起動後、再びGRUBメニュー（Try or Install Ubuntu）が表示される

#### 原因
USB SSD（インストールメディア）を抜くタイミングを逃した。
インストールメディアが接続されたまま再起動したため、USBから起動してしまった。

#### 解決方法

1. PCの電源を切る（電源ボタン長押し）
2. USB SSDを取り外す
3. 再度電源を入れる
4. 内蔵SSDにインストールされたUbuntuが起動する

> **Note**: 正しくは「Please remove the installation medium, then press ENTER」と
> 表示されたタイミングでUSBを抜くが、このメッセージは一瞬で消える場合がある。

---

### HOST-004: Ubuntu起動後にネットワーク接続が切れる

#### 発生日時
2026-01-30

#### エラー内容
Ubuntuのインストール完了後、起動してしばらくするとネットワーク接続が切れる。

#### 発生状況
- Ubuntu 24.04をIntel NUCにインストール後
- 起動直後はネットワークに接続できていたが、途中で切断された

#### 原因
- ネットワークドライバの問題
- NetworkManagerの一時的な不具合
- DHCPリースの問題

#### 解決方法

**方法1: 物理的な再接続**
- 有線LANの場合: ケーブルを抜き差しする
- Wi-Fiの場合: ルーターの電源を再起動

**方法2: NetworkManagerの再起動**
```bash
sudo systemctl restart NetworkManager
```

**方法3: ネットワークインターフェースの確認と再取得**
```bash
# インターフェースの状態確認
ip link

# DHCPでIPアドレスを再取得
sudo dhclient
```

**方法4: Wi-Fiの再接続**
```bash
# 利用可能なWi-Fiネットワークを表示
nmcli device wifi list

# Wi-Fiに接続
nmcli device wifi connect "SSID名" password "パスワード"
```

**方法5: GUIから再接続**
1. 画面右上のネットワークアイコンをクリック
2. 「接続を切断」をクリック
3. 再度ネットワークを選択して接続

---

### HOST-005: SSH接続が「Connection timed out」でタイムアウトする

#### 発生日時
2026-01-30

#### エラー内容
```
ssh: connect to host 192.168.11.18 port 22: Connection timed out
```

#### 発生状況
CursorやVS CodeのリモートSSH機能でIntel NUCに接続しようとした際に発生。

#### 原因
以下のいずれかが考えられる:

1. **PCがスリープモードになっている**（今回の原因）
2. IPアドレスが変わった
3. SSHサービスが停止している
4. ネットワーク接続が切れている

#### 解決方法

**1. スリープモードの確認と解除**

PCがスリープ状態の場合、キーボードを押すかマウスを動かして復帰させる。

**2. スリープを無効にする（推奨）**

開発用PCではスリープを無効にしておくと、SSH接続が安定する。

GUI設定:
1. **設定** → **電源** を開く
2. **画面のブランク** → 「しない」
3. **自動サスペンド** → 「オフ」

コマンドで設定:
```bash
# スリープを無効化
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# 設定確認
systemctl status sleep.target
```

**3. IPアドレスの確認**

Intel NUC側で確認:
```bash
hostname -I
```

**4. SSHサービスの確認**

```bash
sudo systemctl status ssh
```

---

### HOST-006: SSH接続で「REMOTE HOST IDENTIFICATION HAS CHANGED」エラー

#### 発生日時
2026-01-30

#### エラー内容
```
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@    WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!     @
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
IT IS POSSIBLE THAT SOMEONE IS DOING SOMETHING NASTY!
...
Host key verification failed.
```

#### 発生状況
以前同じIPアドレスのホストにSSH接続したことがあり、その後OSを再インストールした場合に発生。

#### 原因
OSを再インストールするとSSHホストキーが新しく生成される。
クライアント側（Windows）のknown_hostsファイルには古いホストキーが保存されているため、
セキュリティ警告として表示される。

#### 解決方法

**Windows側で古いホストキーを削除する**

PowerShellまたはコマンドプロンプトで:

```cmd
ssh-keygen -R 192.168.11.18
```

または手動で編集:

```cmd
notepad C:\Users\ユーザー名\.ssh\known_hosts
```

該当するIPアドレスの行を削除して保存。

その後、再度SSH接続を試みると、新しいホストキーの確認を求められるので「yes」を入力。

---

### HOST-007: CursorリモートSSHでサーバーダウンロードが失敗する

#### 発生日時
2026-01-30

#### エラー内容
```
Unable to establish SSL connection.
Download failed: Error downloading server from https://downloads.cursor.com/production/.../cursor-reh-linux-x64.tar.gz
```

#### 発生状況
CursorのリモートSSH機能でIntel NUC（Ubuntu 24.04）に接続を試みた際に発生。
SSH接続自体は成功するが、Cursor Serverのダウンロードに失敗する。

#### 原因
Ubuntu側でSSL/HTTPS接続に問題がある。以下のいずれかが考えられる:
- ca-certificates（SSL証明書）パッケージがインストールされていない
- SSL証明書が古い
- wgetのSSL設定の問題

#### 解決方法（未検証）

**1. SSL証明書パッケージをインストール**
```bash
sudo apt update
sudo apt install -y ca-certificates
```

**2. 証明書を更新**
```bash
sudo update-ca-certificates
```

**3. HTTPS接続をテスト**
```bash
wget --spider https://downloads.cursor.com
```

成功すれば「200 OK」が表示される。

**4. もし失敗する場合**
```bash
# wgetとcurlを再インストール
sudo apt install -y wget curl

# curlでテスト
curl -I https://downloads.cursor.com
```

その後、Cursorで再度リモートSSH接続を試みる。

#### ステータス
**未解決** - 次回作業時に検証予定

---

### HOST-008: Wi-Fi接続でSSHが断続的に切断される

#### 発生日時
2026-01-31

#### エラー内容
```
Setting up SSH Host: Reconnecting to the remote host (attempt 7)...
```

SSH接続が確立しても数秒〜数十秒で切断され、再接続を繰り返す。
TeraTermやコマンドラインからのSSH接続も同様に不安定。

#### 発生状況
- Intel NUC（Ubuntu 24.04）にWi-Fi接続している環境
- CursorやVS CodeのリモートSSH機能、またはTeraTermでSSH接続
- 接続直後は成功するが、すぐに切断される
- 「Reconnecting to the remote host (attempt N)...」が繰り返し表示される

#### 原因
**Wi-Fiの省電力機能（Power Save）** が原因。

Ubuntuのデフォルト設定では、Wi-Fiアダプタの省電力機能が有効になっている。
この機能により、通信がない状態が続くとWi-Fiアダプタがスリープ状態に入り、
SSH接続のキープアライブパケットが途切れて接続が切断される。

#### 解決方法

**1. Wi-Fi接続名の確認**

```bash
sudo nmcli connection show
```

出力例:
```
NAME                UUID                                  TYPE      DEVICE
Baffalo-5330-WPA3   xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx  wifi      wlan0
```

`NAME` 列の値（例: `Baffalo-5330-WPA3`）が接続名。

**2. Wi-Fi省電力を無効化**

```bash
sudo nmcli connection modify "接続名" 802-11-wireless.powersave 2
```

例:
```bash
sudo nmcli connection modify "Baffalo-5330-WPA3" 802-11-wireless.powersave 2
```

> **Note**: `802-11-wireless.powersave` の値の意味:
> - `0`: デフォルト（プロファイルに従う）
> - `1`: 無視（省電力設定を無視）
> - `2`: 無効（省電力を完全に無効化）← **推奨**
> - `3`: 有効（省電力を有効化）

**3. 接続を再起動して設定を反映**

```bash
sudo nmcli connection down "接続名" && sudo nmcli connection up "接続名"
```

例:
```bash
sudo nmcli connection down "Baffalo-5330-WPA3" && sudo nmcli connection up "Baffalo-5330-WPA3"
```

**4. 設定の確認**

```bash
nmcli connection show "接続名" | grep powersave
```

出力例:
```
802-11-wireless.powersave:              2 (disable)
```

`2 (disable)` と表示されれば設定完了。

#### 補足: iwコマンドでの確認（任意）

`iw` コマンドがインストールされている場合、現在の省電力状態を確認できる:

```bash
# iwコマンドのインストール（必要な場合）
sudo apt install -y iw

# 省電力状態の確認
iw dev wlan0 get power_save
```

出力:
- `Power save: on` → 省電力が有効（問題あり）
- `Power save: off` → 省電力が無効（正常）

#### 代替解決策: 有線LAN接続

Wi-Fiの問題を根本的に回避するには、**有線LAN接続に切り替える**ことを推奨。
Intel NUCには有線LANポートがあり、有線接続では省電力による切断は発生しない。

#### ステータス
**解決済み** - Wi-Fi省電力無効化で安定

---

## エラー記載テンプレート

```markdown
### HOST-NNN: エラータイトル

#### 発生日時
YYYY-MM-DD

#### 発生環境
- PC:
- OS:
- その他関連情報

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
| 2026-01-31 | HOST-008: Wi-Fi接続でSSHが断続的に切断される問題を追加（解決済み） |
| 2026-01-30 | 初版作成 |
| 2026-01-30 | HOST-001: BIOS/Legacy Boot of UEFI-only mediaエラーを追加 |
| 2026-01-30 | HOST-002: Intel NUC Visual BIOSの操作方法を追加 |
| 2026-01-30 | HOST-003: インストール後に再びインストールメニューが表示される問題を追加 |
| 2026-01-30 | HOST-004: ネットワーク接続が切れる問題を追加 |
| 2026-01-30 | HOST-005: SSH接続タイムアウト（スリープモード）の問題を追加 |
| 2026-01-30 | HOST-006: SSHホストキー変更エラーの問題を追加 |
| 2026-01-30 | HOST-007: CursorリモートSSHでサーバーダウンロード失敗の問題を追加 |
