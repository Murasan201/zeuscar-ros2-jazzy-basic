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

---

## 2. ROS2 Jazzyのインストール

### 2.1 ロケール設定
（手順を記載予定）

### 2.2 リポジトリの追加
（手順を記載予定）

### 2.3 ROS2パッケージのインストール
（手順を記載予定）

### 2.4 環境設定
（手順を記載予定）

---

## 3. 開発環境のセットアップ

### 3.1 必要パッケージのインストール
（手順を記載予定）

### 3.2 colconのインストール
（手順を記載予定）

### 3.3 rosdepのセットアップ
（手順を記載予定）

---

## 4. プロジェクト固有のセットアップ

### 4.1 リポジトリのクローン
（手順を記載予定）

### 4.2 依存関係のインストール
（手順を記載予定）

### 4.3 ビルド
（手順を記載予定）

---

## 5. ハードウェアセットアップ

### 5.1 Arduinoの接続
（手順を記載予定）

### 5.2 LiDARの接続
（手順を記載予定）

### 5.3 udevルールの設定
（手順を記載予定）

---

## 6. 動作確認

### 6.1 ROS2の動作確認
（手順を記載予定）

### 6.2 シリアル通信の確認
（手順を記載予定）

### 6.3 LiDARの動作確認
（手順を記載予定）

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2026-01-25 | 初版作成（テンプレート） |
| 2026-01-25 | 1.1〜1.3 OS環境セットアップ手順を追加 |
