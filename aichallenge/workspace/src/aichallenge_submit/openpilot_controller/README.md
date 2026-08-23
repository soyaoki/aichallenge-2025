# openpilot_controller

comma.ai の [openpilot](https://github.com/commaai/openpilot) の運転モデル
`driving_supercombo.onnx` を、AI チャレンジのカメラ入力で動かす End-to-End 制御ノード。

`/sensing/camera/image_raw` を openpilot のモデル座標系にワープして推論し、出力された
plan から desired curvature / acceleration を求めて `/control/command/control_cmd` に出す。

- 上流: `commaai/openpilot@084747c7` (MIT、`UPSTREAM_LICENSE`)
- 移植元と改変内容: `THIRD_PARTY_NOTICES.md`
- 設計・検証結果・既知の落とし穴: [docs/spec/openpilot-integration.md](../../../../../docs/spec/openpilot-integration.md)

## 使い方

```bash
make openpilot-models     # モデル重み (58 MB) を取得。ビルド前に一度だけ
make autoware-build
make simulator-openpilot  # 1 台 / NPC なし / カメラ有効のプリセット

CONTROL_METHOD=openpilot docker compose up -d autoware
```

モデル重みは**リポジトリにコミットしていない**。提出用 tar や評価イメージのビルド前に
`make openpilot-models` を実行しておくこと (評価実行時のネットワークアクセスは不要)。

## 立ち上げの順序

**既定では車両制御を行わない。** まずモデルがコースを見えているかを確認してから制御をつなぐ。

```bash
# 1. 観測のみ。RViz で /openpilot/debug/markers と /openpilot/debug/image を見る
CONTROL_METHOD=openpilot docker compose up -d autoware

# 2. モデルの出力で走らせる
CONTROL_METHOD=openpilot OPENPILOT_CONTROL_ENABLED=true docker compose up -d autoware

# 3. モデルを planner として使い、追従は pure_pursuit に任せる
CONTROL_METHOD=openpilot OPENPILOT_OUTPUT_MODE=trajectory docker compose up -d autoware
```

1 の合格条件は「走ること」ではなく、**コーナーに入る前に予測経路がコース方向へ曲がること**。

`/openpilot/debug/image` の `output.debug_image_source` は `model_input` (既定、モデルが
実際に食っているワープ画像) と `camera` (元画像に逆投影) を選べる。

## 主なパラメータ

`config/openpilot_controller.param.yaml` を参照。調整が必要になりやすいのは以下。

| パラメータ | 内容 |
|---|---|
| `control.enabled` | `false` の間は control_cmd を publish しない (既定) |
| `output.mode` | `direct_action` / `trajectory` |
| `model.provider` | `auto` / `cpu` / `cuda` / `tensorrt` |
| `camera.calib_pitch` | カメラ取り付けピッチ [rad]。`auto_calibration` で走行中に推定もできる |
| `camera.hfov_deg` | `camera_info` が来ない場合のフォールバック画角 |
| `control.launch_speed` | 発進補助。モデルは停止中に停止を返すため必要 |
| `control.max_curvature` | 既定 0.44 はカートの運動学限界。openpilot 既定は 0.2 |
| `vehicle.steering_tire_angle_gain` | AWSIM 用に 1.5 (`simple_pure_pursuit` と同値) |

環境変数 `OPENPILOT_PROVIDER` / `OPENPILOT_CALIB_PITCH` / `OPENPILOT_STEERING_GAIN` /
`OPENPILOT_LAUNCH_SPEED` / `OPENPILOT_CALIB_MIN_SPEED` でも上書きできる。

## 動作確認 (シミュレータ不要)

```bash
# モデルの有無・チェックサム・provider・推論レート
python3 scripts/smoke_test.py

# 統合が壊れていないことを実データで確認 (初回のみ 37 MB DL)
python3 scripts/verify_against_real_footage.py

# AWSIM のスクショ 1 枚から重畳画像を描いて画角・ピッチを詰める
python3 scripts/render_debug_image.py --image frame.png --calib-pitch 0.03
```

`verify_against_real_footage.py` は openpilot 公開セグメントで車線検出を確認し、
自車線の確率が 0.5 を下回ると失敗終了する。AWSIM の中からでは前処理のバグと
「モデルがこのシーンを苦手」が同じ症状に見えるため、これで切り分ける。

## GPU

`docker-compose.gpu.yml` が `OPENPILOT_PROVIDER=cuda` を設定する。onnxruntime の
CUDA provider は cuDNN 9 を要求し、本リポジトリの PyTorch (cu121) が pin する cuDNN 8 と
衝突するため、独立したディレクトリに入れて launch がこのノードにだけパスを通す。

```bash
make openpilot-gpu-deps   # /opt/openpilot-cudnn9 を用意して dev イメージを再ビルド
```

未実行のまま `cuda` を指定すると初期化に失敗する。`auto` なら CPU にフォールバックし、
選ばれた provider は起動時にログへ出る。

## 既知の制約

**コースを周回しない。** 走りはするが壁に接触して止まる。原因は実装ではなく、モデルが
カート用バリアと路面標示の無いコースを道路として認識していないこと。実車映像に対しては
車線を確度 0.9 以上・標準偏差 0.13 m で検出する (`verify_against_real_footage.py`)。
カメラ解像度・画角・取り付け角・内部パラメータの解釈はいずれも要因ではないことを確認済み。

詳細と実測値は [docs/spec/openpilot-integration.md](../../../../../docs/spec/openpilot-integration.md) を参照。
