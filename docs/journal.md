# 開発ジャーナル

## 2026-01-25

### 作業内容

#### プロジェクト初期設定
- [x] `.claude/settings.json` 作成（Claude Code許可ツール設定）
- [x] `CLAUDE.md` 作成（プロジェクトルールファイル）
- [x] `.gitignore` 作成（reference/等を除外）
- [x] `reference/` ディレクトリ作成
- [x] `docs/` ディレクトリ作成

#### ドキュメント作成
- [x] `docs/requirements.md` - 要件定義書
- [x] `docs/system-specification.md` - システム仕様書
- [x] `docs/setup-guide.md` - セットアップガイド
- [x] `docs/troubleshooting.md` - トラブルシューティング
- [x] `docs/backlog-raspberry-pi.md` - Raspberry Piバックログ
- [x] `docs/README.md` - ドキュメント索引

#### ルール設定
- ドキュメントは`docs/`に集約
- `reference/`は参照専用（変更禁止）
- 作業完了後はコミット
- プッシュ前に機密情報チェック
- プッシュ前にユーザー確認必須
- セットアップ手順は全て記録
- エラーと解決方法は全て記録
- ドキュメント追加時は索引更新

#### 環境構築（バックログ1.1 OS環境）
- [x] RPI-001: Ubuntu 24.04 LTSのインストール確認
- [x] RPI-002: システムアップデート（apt-get update/upgrade）
- [x] RPI-003: 必要なシステムパッケージのインストール
  - build-essential, cmake, git, wget, curl, vim, python3-pip, software-properties-common
- [x] RPI-021: Python開発環境の確認
- [x] RPI-022: C++開発環境の確認

#### Git設定
- ユーザー名: Murasan201
- メール: shinf0330@gmail.com
- リモートURLにトークン設定済み

### コミット履歴
1. `73a492a` - プロジェクト初期設定と要件定義書を追加
2. `16d47d4` - 機密情報確認ルールを追加
3. `aa80a33` - システム仕様書を追加
4. `00874a6` - プッシュ前の確認ルールを追加
5. `a49c389` - セットアップガイドとトラブルシューティングを追加
6. `4c11f8a` - Raspberry Piバックログを追加
7. `264770b` - ドキュメント索引を追加
8. `bad9c4f` - OS環境セットアップ完了（RPI-001〜003, RPI-021〜022）

---

## 次回作業開始時にやること

### 1. 未プッシュのコミット確認
```bash
git status
git log origin/main..HEAD --oneline
```

### 2. 前回の作業をプッシュ
```bash
git push origin main
```

### 3. バックログ確認
次のタスクは **1.2 ROS2環境** です。

| ID | タスク |
|----|--------|
| RPI-010 | ROS2 Jazzyのインストール |
| RPI-011 | ROS2環境変数の設定 |
| RPI-012 | colconのインストール |
| RPI-013 | rosdepのセットアップ |
| RPI-014 | ROS2動作確認 |

### 4. セットアップガイドを開く
```bash
# セットアップガイドの続きを記載しながら作業
docs/setup-guide.md
```

### 5. ROS2 Jazzyインストール手順
ROS2公式ドキュメントを参照:
- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

主な手順:
1. ロケール設定
2. ROS2リポジトリの追加
3. ROS2パッケージのインストール（ros-jazzy-desktop）
4. 環境変数の設定（~/.bashrc）
5. colconのインストール
6. rosdepの初期化
