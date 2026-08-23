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

## 段階的な立ち上げ

**既定では車両制御を行わない** (`control.enabled: false`)。まずモデルが AWSIM の
コースを「見えているか」を確認し、それから制御をつなぐ。

### Phase 1: 予測経路の確認 (制御オフ)

```bash
ros2 launch ... control_method:=openpilot     # control_enabled は既定 false
```

RViz で `/openpilot/debug/markers` (`visualization_msgs/MarkerArray`, `base_link`) を表示する。

| namespace | 内容 |
|---|---|
| `path` | 予測経路 (緑, 33 点 / 0〜10 秒) |
| `lane_lines` | 車線 4 本。透明度が確率 |
| `road_edges` | 路端 2 本 |

さらに `/openpilot/debug/image` (`sensor_msgs/Image`, `bgr8`) に、予測を画像へ
重畳したものを publish する。RViz の Image ディスプレイか `rqt_image_view` で見る。

| `output.debug_image_source` | 内容 |
|---|---|
| `model_input` (既定) | **モデルが実際に食っている 512x256 のワープ画像**。画角・取り付けピッチのズレが最初に出るのはここ |
| `camera` | 元のカメラ画像に予測を逆投影したもの |

plan は 2D ではなくキャリブレーション座標系の 3D 点、lane_lines / road_edges は
固定 `X_IDXS` 上の (y, z) 断面なので、カメラ内部パラメータで逆投影して重畳できる。
経路のみカメラ高さ (`camera.height`) 分だけ下げて路面に乗せている (openpilot 本体の
renderer と同じ扱い)。

負荷が気になる場合は `output.debug_image_decimation` で間引くか
`output.publish_debug_image: false` で止める。

**合格条件は「走ること」ではなく、コーナーに入る前に予測経路がコース方向へ曲がること。**
ここが通らないうちに操舵をつないでも意味がない。曲がらない場合はまず
`camera.calib_pitch` と `camera.hfov_deg` を疑う。

### Phase 2: 制御に接続

```bash
# 環境変数でも切り替えられる (docker compose 経由)
CONTROL_METHOD=openpilot OPENPILOT_CONTROL_ENABLED=true docker compose up -d autoware

# A. モデルの action を直接 control_cmd にする
ros2 launch ... control_method:=openpilot openpilot_control_enabled:=true

# B. モデルを planner として使い、追従は既存コントローラに任せる
ros2 launch ... control_method:=openpilot openpilot_output_mode:=trajectory
```

`trajectory` モードでは予測経路を `autoware_auto_planning_msgs/Trajectory`
(`base_link` 系) として `/openpilot/trajectory` に publish する。追従させるには
このトピックを対象コントローラの入力へ remap し、そのコントローラを別途起動する。
このモードでは `control_cmd` は publish しない。

## 座標系

comma の device frame は x 前 / y 右 / z 下、`base_link` は x 前 / y 左 / z 上 なので、
既定では y と z を反転している。実走で符号を確認できるよう設定可能にしてある。

```yaml
frame:
  swap_xy: false
  invert_x: false
  invert_y: true
  invert_z: true
```

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
python3 .../scripts/smoke_test.py
python3 .../scripts/smoke_test.py --provider cuda   # GPU 側の確認
```

AWSIM のスクリーンショット 1 枚から重畳画像を描いて、画角・ピッチを詰める。

```bash
python3 .../scripts/render_debug_image.py --image frame.png --out /tmp
python3 .../scripts/render_debug_image.py --image frame.png --calib-pitch 0.03
```

静止画を繰り返し入力するため 2 フレーム間の動きがゼロになり、モデルは「停止中」と
判断して経路が極端に短くなる。この用途で見るのは**地平線の位置と車線・路端の当たり**。

## 主なパラメータ

`config/openpilot_controller.param.yaml` を参照。調整が必要になりやすいのは以下。

| パラメータ | 内容 |
|---|---|
| `camera.hfov_deg` | `/camera_info` が来ない場合に仮定する水平画角。AWSIM のカメラに合わせる |
| `camera.calib_pitch` | カメラの取り付けピッチ [rad]。地平線位置がずれると操舵が偏るので最初に調整する |
| `camera.height` | カメラの路面からの高さ [m]。重畳画像で経路を路面に落とすためだけに使う |
| `output.debug_image_source` | `model_input` / `camera` |
| `control.enabled` | `false` の間は control_cmd を publish しない (既定) |
| `output.mode` | `direct_action` / `trajectory` |
| `control.max_speed` | plan 由来の目標速度の上限 [m/s] |
| `control.max_lateral_accel` | openpilot 由来の横加速度上限 [m/s^2]。市販車向けの値なのでカートには保守的 |
| `control.apply_curvature_limits` | 上記の安全包絡線を無効化する場合は `false` |
| `vehicle.wheel_base` | 曲率 → 操舵角の変換に使う。racing kart は 1.087 m |

## AWSIM で走らせるときの注意

`make simulator-openpilot` (`aichallenge/simulator_scripts/openpilot.sh`) が
1台・NPC なし・カメラ有効の最小構成。立ち上げで踏んだ落とし穴:

- **AWSIM は `longitudinal.speed` を追従する。** acceleration だけ出しても動かない。
  停止状態ではモデルが shouldStop を返すので目標速度が 0 のままになり、永久に
  発進しない。`control.launch_speed` / `launch_acceleration` がこれを破る。
- **壁に当たると `--wall-recovery off` では永久に固着する。** 立ち上げ中に
  「動かない」の切り分けが不可能になるので、このプリセットは復帰を有効にしている。
- **AWSIM のカメラは 384x256・約 10 Hz** で、openpilot 実機 (1928x1208 / 20 Hz) から
  大きく外れる。内部パラメータは `camera_info` から取れる (fx=192, fy=227.4)。
  モデルが積む 2 フレームの間隔は `model.frame_skip` で調整され、既定 (0) では
  実測の処理レートから自動で決まる。
- GPU を使う場合は `make openpilot-gpu-deps` でイメージを再ビルドしておく
  (onnxruntime の CUDA provider には cuDNN 9 と CUDA ランタイムが要る)。
- **デバッグ画像は推論より重い。** 毎フレーム publish するとパイプラインが
  9.5 → 2.3 frames/s まで落ち、モデルが積む 2 フレームの間隔が 435 ms になる。
  既定の `debug_image_decimation: 5` を下げるときは `frames/s` のログを見ること。

## キャリブレーション

`camera.auto_calibration` を有効にすると、openpilot の `calibrationd` と同じ方法で
取り付け角を走行中に推定する。モデルが出す自車並進 (`pose[:3]`) の向きが本来
カメラ光軸と一致するはずなので、そのズレが取り付け誤差になる。

```
observed = [0, -atan2(trans_z, trans_x), atan2(trans_y, trans_x)]
```

直進かつ `calibration_min_speed` 以上、ヨーレート `calibration_max_yaw_rate_deg`
未満のフレームのみ採用し、100 フレーム × 5 ブロックで収束する。

他のコントローラに運転させながら観測だけさせられる (実機の openpilot が人間の運転中に
キャリブレするのと同じ)。結果は `calibration_file` に保存され次回起動時に読まれる。

```bash
OPENPILOT_OBSERVE=true docker compose up -d autoware   # MPC が走り openpilot は観測のみ
```

**ただし citycircuit では収束しない。** 実測 (MPC 運転、120 フレーム):

| ゲート | 通過率 |
|---|---|
| `v_ego > 3.0` | 33 % |
| 予測 `trans_x > 3.0` | 40 % |
| ヨーレート < 2 deg/s | 55 % (ほぼ停止中のフレーム) |
| 3 つ同時 | **0 %** |

走行中のヨーレート中央値は 24.7 deg/s で、サーキットに openpilot が要求する
直進区間が存在しない。`calibration_max_yaw_rate_deg` を上げれば推定は始まるが、
旋回中のフレームは並進方向がカメラ光軸と一致しないので推定が偏る。
収束状況は 20 秒ごとに `calibration: ...` としてログに出る。

手で詰める場合は `scripts/render_debug_image.py --calib-pitch` で AWSIM の
スクショに対して掃引する。

## 統合が壊れていないことの確認

AWSIM の中からでは、ワープのバグも出力 slice のズレも「モデルが何も出さない」と
いう同じ症状に見える。内部パラメータが確定している実データに同じコードを流せば
切り分けられる (初回のみ 37 MB ダウンロード)。

```bash
python3 .../scripts/verify_against_real_footage.py
```

自車線の車線確率が 0.5 を下回ったら失敗として終了するので、前処理を壊す変更が
「難しいシーンだった」で見逃されない。

## 既知の制約

- openpilot は road カメラと wide カメラの 2 系統を前提とするが、ここでは 1 台の
  カメラを両方の入力に流用している。`big_img` 側は本来より狭い画角の映像になるため、
  周辺は端の画素で埋められる。
- モデルは実車の走行映像で学習されており、AWSIM のレース環境はドメイン外。
  カメラ内部パラメータと取り付け角を合わせないと妥当な出力にならない。
- desire 入力は常にゼロ (車線変更プランナは無い)。
- 実走ではコースを外れて壁に接触する。**発進アシストをモデルに引き渡す形に直した後**の
  実測は 90〜120 秒で 22〜87 m、停止状態が時間の 9 割近くを占める。走行のばらつきが
  大きい。

  発進アシストを恒久的な速度下限にすると 155〜174 m まで伸びるが、それは
  `launch_speed` で自分が押しているだけでモデルの走行ではない。既定は
  「発進したら引き渡す」にしてある (`control.launch_rearm_sec`)。

- **パイプラインは正しい (実データで検証済み)。**
  `scripts/verify_against_real_footage.py` が openpilot の CI 公開セグメント
  (neo カメラ 1164x874 / f=910 / 20 fps、内部パラメータが確定している) を落として
  同じ経路に流す。結果は車線確率 [0.90, 0.99, 0.97, 0.69]、自車線平均 0.98、
  車線の標準偏差 0.13 m (最良 0.06 m) ＝ openpilot が正常動作しているときの水準。
  同じコードで AWSIM は車線確率 0.02、標準偏差 4.9 m で **平均 60 倍の差**。
  ワープ・YUV パッキング・時系列スタック・出力 slice はすべて正しい。
- **限界要因は解像度ではなくシーンの中身。** 実写道路を段階的に粗くしても車線確率は
  保たれる (0.048 deg/px で 0.13、AWSIM より粗い 0.250 deg/px でも 0.15) のに対し、
  AWSIM は 0.234 deg/px で 0.020。同じ粗さで 6〜13 倍の差が残る。AWSIM のカメラ
  解像度はビルドに焼き込まれていて変更できないが、上げても解決しない。
- 内部パラメータの想定 (AWSIM 申告の非正方画素 fx=192/fy=227.4 か、正方画素か) を
  変えても結果は同一。
- **自車運動の推定は効いている。** 録画した AWSIM シーケンス (9.5 Hz, 120 フレーム) を
  流すと、予測前進速度は実測 5.25 → 5.39、3.57 → 3.37、停止中は 0.00 と追従し、
  平均絶対誤差 1.43 m/s。ワープ・時系列スタック・パース自体は機能している。
- **一方で車線・路端の意味理解は出ていない。** 車線確率は 0.00〜0.04、路端の
  標準偏差は 22.7 m 先で 5.7〜7.6 m と道路幅より大きい。左右反転しても路端は
  符号反転しない (事前分布に近い)。オプティカルフロー由来の自車運動は
  ドメインを跨いで効くが、車線の意味論は効かない、という切り分け。
- 固定した `driving_supercombo.onnx` の `output_slices` には `action` 出力が**無い**。
  そのため openpilot 本体と同じく `plan` から desired curvature / acceleration を
  導出している (`get_curvature_from_plan` / `get_accel_from_plan`)。
