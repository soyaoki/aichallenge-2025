# openpilot_controller

comma.ai の [openpilot](https://github.com/commaai/openpilot) の運転モデル
`driving_supercombo.onnx` を、AI チャレンジのカメラ入力で動かす End-to-End 制御ノード。

`/sensing/camera/image_raw` を openpilot のキャリブレーション済みモデル座標系に
ワープしてモデルへ入力し、出力された plan から desired curvature / acceleration を
求めて `/control/command/control_cmd` を publish する。

- 上流コミット: `commaai/openpilot@084747c7`
- ライセンス: MIT (`UPSTREAM_LICENSE`)。移植元と改変内容は `THIRD_PARTY_NOTICES.md` を参照

## 使い方

```bash
# 1. モデル重み (58 MB) を取得。ビルド前に一度だけ必要
make openpilot-models

# 2. ビルド
make autoware-build

# 3. カメラ有効なシミュレータと一緒に起動
make simulator-e2e
make autoware-simulator
```

`control_method:=openpilot` で選択される。既定の `control_method` は `mpc` のままなので、
`reference.launch.xml` の既定値を変えるか launch 引数で明示的に渡すこと。

モデル重みは**リポジトリにコミットしていない**。提出用 tar (`create_submit_file.bash`) や
評価用イメージのビルド前に `make openpilot-models` を実行しておく必要がある
(評価実行時のネットワークアクセスは不要)。

## 推論プロバイダ (CPU / GPU)

`model.provider` パラメータで切り替える。

| 値 | 動作 |
|---|---|
| `auto` (既定) | TensorRT → CUDA → CPU の順に、利用可能なものを選ぶ |
| `cpu` | CPU 実行 |
| `cuda` | CUDA。使えなければ起動時にエラー |
| `tensorrt` | TensorRT。無ければ CUDA、CPU の順にフォールバック |

環境変数 `OPENPILOT_PROVIDER` でも上書きでき、`docker-compose.gpu.yml` はこれを `cuda` に設定する。

```bash
# CPU を明示
OPENPILOT_PROVIDER=cpu docker compose up -d autoware
# GPU オーバーレイ
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d autoware
```

実際に選ばれたプロバイダは起動時に
`driving_supercombo loaded (providers=[...])` としてログに出る。CPU でも 1 推論
約 20〜30 ms (30〜50 Hz) で、モデルの想定レート 20 Hz には十分間に合う。

### CUDA を使う場合の追加セットアップ

`requirements.txt` が入れる `onnxruntime-gpu` の CUDA provider は cuDNN 9 を要求するが、
本リポジトリの PyTorch (cu121) は cuDNN 8 を pin している。両者が衝突しないよう、
cuDNN 9 は独立したディレクトリに入れ、launch がこのノードのプロセスにだけ
`LD_LIBRARY_PATH` を通す。

```bash
make openpilot-gpu-deps   # /opt/openpilot-cudnn9 に cuDNN 9 を配置
```

未実行のまま `provider=cuda` にすると CUDA provider の初期化に失敗する。`auto` なら
CPU にフォールバックし、その旨がログに残る。

## 動作確認 (シミュレータ不要)

モデルの有無・チェックサム・provider の選択・推論レートをまとめて確認する。

```bash
python3 aichallenge/workspace/src/aichallenge_submit/openpilot_controller/scripts/smoke_test.py
python3 .../scripts/smoke_test.py --provider cuda   # GPU 側の確認
```

## 主なパラメータ

`config/openpilot_controller.param.yaml` を参照。調整が必要になりやすいのは以下。

| パラメータ | 内容 |
|---|---|
| `camera.hfov_deg` | `/camera_info` が来ない場合に仮定する水平画角。AWSIM のカメラに合わせる |
| `camera.calib_pitch` | カメラの取り付けピッチ [rad]。地平線位置がずれると操舵が偏るので最初に調整する |
| `control.max_speed` | plan 由来の目標速度の上限 [m/s] |
| `control.max_lateral_accel` | openpilot 由来の横加速度上限 [m/s^2]。市販車向けの値なのでカートには保守的 |
| `control.apply_curvature_limits` | 上記の安全包絡線を無効化する場合は `false` |
| `vehicle.wheel_base` | 曲率 → 操舵角の変換に使う。racing kart は 1.087 m |

## 既知の制約

- openpilot は road カメラと wide カメラの 2 系統を前提とするが、ここでは 1 台の
  カメラを両方の入力に流用している。`big_img` 側は本来より狭い画角の映像になるため、
  周辺は端の画素で埋められる。
- モデルは実車の走行映像で学習されており、AWSIM のレース環境はドメイン外。
  カメラ内部パラメータと取り付け角を合わせないと妥当な出力にならない。
- desire 入力は常にゼロ (車線変更プランナは無い)。
