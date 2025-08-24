# AI Challenge 2025 - Vehicle Setup

## Makefile使い方

### 基本的なサービス起動・ログ表示

#### 個別サービス起動
```bash
# Autowareのみ起動・ログ表示
make autoware

# Racing Kartドライバー起動・ログ表示
make driver

# Zenoh通信サービス起動・ログ表示
make zenoh

# RViz2起動・ログ表示（前回のプロセスを停止してから起動）
make rviz2

# ROSBag記録起動・ログ表示
make rosbag
```

#### フルシステム起動
```bash
# Autoware + Driver + Zenohを一度に起動（ERROR/WARNのみ表示、全ログはfull_log.logに保存）
make run-full-system
```

### ビルド関連

```bash
# Autowareのビルド
make build-autoware

# Driverのビルド
make build-driver
```

### デバッグ・開発

```bash
# Racing Kart InterfaceコンテナでBashシェル起動
make driver-bash
```

### データダウンロード

```bash
# 最新のsubmissionデータをダウンロード
make download

# 特定のsubmission IDを指定してダウンロード
make download SUBMISSION_ID=<id>
```

### 統合コマンド

```bash
# ダウンロード→ビルド→実行を一括実行
# Note: "download-only"依存関係があります
make all
```

### サービス停止

```bash
# ROSBag停止 + 全コンテナ停止（5秒待機後）
make stop-rosbag

# 全コンテナ停止
docker compose down
```

### ログ出力について

- `run-full-system`では、ERROR・WARNメッセージのみを画面表示し、全ログを`full_log.log`ファイルに同時保存します
- 個別サービスでは全ログが表示されます
- `tee`コマンドでログを分岐しているため、リアルタイム表示と保存が同時実行されます

### ログファイル

```bash
# 保存されたログファイルを確認
cat full_log.log

# ログファイルをリアルタイムで監視
tail -f full_log.log
```

### トラブルシューティング

1. **コンテナ名の競合エラーが出る場合**
   ```bash
   docker compose down
   docker ps -a | grep -E "(racing_kart_interface|aic-2025)" | awk '{print $1}' | xargs docker rm -f
   ```

2. **ログを詳細に確認したい場合**
   ```bash
   # 保存されたログファイルを確認
   cat full_log.log
   
   # リアルタイムで全ログを見る（フィルタなし）
   docker compose logs -f autoware driver zenoh
   ```

3. **特定のサービスのみ再起動**
   ```bash
   docker compose restart <service_name>
   ```

4. **ビルドが失敗する場合**
   ```bash
   # 古いコンテナを削除してからビルド
   docker compose down
   docker system prune -f
   make build-autoware
   make build-driver
   ```

### 使用例

```bash
# 1. 初回セットアップ
make download
make build-autoware
make build-driver
make run-full-system

# 2. 開発時の通常起動
make run-full-system

# 3. デバッグ時（複数ターミナル使用）
make driver-bash     # ターミナル1: デバッグ用シェル
make autoware        # ターミナル2: Autoware単体起動
make zenoh          # ターミナル3: Zenoh単体起動

# 4. 新しいsubmissionデータでテスト
make stop-rosbag    # 既存システム停止
make download       # 新データダウンロード
make build-autoware # リビルド
make run-full-system # 実行

# 5. 可視化付きデバッグ
make build-autoware
make build-driver
make run-full-system  # メインシステム（別ターミナル）
make rviz2           # 可視化
```

### 依存関係

- `all` → `download-only`
- `rviz2` → 前回のrviz2プロセスを自動停止
- `stop-rosbag` → rosbag停止後に全コンテナ停止
