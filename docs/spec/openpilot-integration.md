# openpilot_controller インテグレーション設計

> 仕様ドキュメント（現仕様の正）。最終確認: 2026-08-23。文書運用方針は [docs/README.md](../README.md) を参照。

作成日: 2026-08-23

## 概要

comma.ai [openpilot](https://github.com/commaai/openpilot) の運転モデル `driving_supercombo.onnx` を
`aichallenge_submit` に統合したもの。`reference.launch.xml` の `control_method:=openpilot` で選択する。

使い方は [openpilot_controller/README.md](../../aichallenge/workspace/src/aichallenge_submit/openpilot_controller/README.md)、
移植元と改変内容は同ディレクトリの `THIRD_PARTY_NOTICES.md` を参照。本書は設計と検証結果を記録する。

## モデル

上流コミット `commaai/openpilot@084747c75d2cbd23af65ab7a9e770bbd7b98bac9`、
`openpilot/selfdrive/modeld/models/driving_supercombo.onnx`（MIT / 60,881,999 bytes / opset 20）。

supercombo は「単一 → vision と policy に分割 → 再統合」と変遷しており、**バージョンにより入出力が異なる**。
本統合は再統合後の master 版を前提とする。重みはリポジトリにコミットせず、
`make openpilot-models` が上流から取得して SHA-256 を検証する。ロード時にも再検証する
（`output_slices` メタデータが pickle のため）。

### 入出力

| 入力 | shape | dtype |
|---|---|---|
| `img` / `big_img` | (1,12,128,256) | uint8 |
| `features_buffer` | (1,24,512) | float16 |
| `desire_pulse` | (1,25,8) | float16 |
| `traffic_convention` | (1,2) | float16 |
| `action_t` | (1,2) | float16 |

出力は `outputs` (1,2576) float16 の一本。ONNX の metadata に base64 pickle で埋め込まれた
`output_slices` で `meta / desire_pred / pose / wide_from_device_euler / road_transform /
lane_lines / lane_lines_prob / road_edges / lead / lead_prob / hidden_state / plan / desire_state`
に分解する。

**この版に `action` 出力は無い。** openpilot 本体も無い場合は `plan` から
`get_curvature_from_plan` / `get_accel_from_plan` で導出するフォールバック経路を通るため、本統合も同じ実装とする。

## パイプライン

```
/sensing/camera/image_raw (384x256 bgr8)
  → RGB → I420 (Y, U/V)
  → warp = K @ view_from_device @ rot(calib) @ calib_from_medmodel
      Y を 512x256、U/V を M*[[1,1,.5],[1,1,.5],[2,2,1]] で 256x128 に
      (INTER_NEAREST + BORDER_REPLICATE = 上流の tinygrad カーネルと同じ)
  → 6ch にパック: Y[0::2,0::2], Y[1::2,0::2], Y[0::2,1::2], Y[1::2,1::2], U, V
  → img = 直近と frame_skip 前の 2 フレームを 12ch に積む
  → 推論
  → plan (33,15) → 曲率 / 加速度 → AckermannControlCommand
```

`big_img` は本来 wide カメラ用。車両にカメラが 1 台しかないため同じフレームを sbigmodel 側の warp で流用する。

### 時系列

モデルは 200 ms 離れた 2 フレームを積む。上流は 20 Hz 固定で `frame_skip=4`。
**AWSIM のカメラは約 10 Hz** のため既定のままでは 400 ms 離れる。
`model.frame_skip: 0`（既定）は実測の処理レートから自動決定し、実環境では 2（210 ms）に収束する。

計測はウォームアップ 40 フレーム後に行う。最初の CUDA 推論は定常より大幅に遅く、
これを含めると過小な値に固定されるため。

## 検証

### 統合の正しさ（実データによる対照実験）

AWSIM の中からでは、ワープのバグも出力 slice のズレも「モデルが何も出さない」という同じ症状になり区別できない。
`scripts/verify_against_real_footage.py` が openpilot CI の公開セグメント
（neo カメラ 1164x874 / f=910 / 20 fps、内部パラメータが確定）を取得して同じ経路に流す。

| | 実車の走行映像 | AWSIM カートコース |
|---|---|---|
| 車線確率（4 本） | 0.90 / 0.99 / 0.97 / 0.69 | 0.00 / 0.01 / 0.02 / 0.00 |
| 車線位置の標準偏差 | 0.13 m（最良 0.06 m） | 4.90 m |
| 路端の標準偏差 | 1.74 m | 9.37 m |

実車映像で openpilot が正常動作する水準に達するため、ワープ・YUV パッキング・時系列スタック・
出力 slice はいずれも正しい。自車線の車線確率が 0.5 を下回るとスクリプトは失敗終了する。

### 前処理の自己整合

- warp 幾何: 地上点をモデル座標系→カメラ座標系に往復させて誤差 0.0000 px
- 輝度チャンネルのパッキング: 直接 warp した Y と完全一致
- フレーム順: ch0-5 が古いフレーム、ch6-11 が新しいフレーム、frame_skip ステップ差
- 自車運動: 録画シーケンスで予測前進速度が実測に平均絶対誤差 1.43 m/s で追従

### 限界要因

AWSIM のカート環境では車線・路端がほぼ検出されない。以下はいずれも要因ではないことを確認済み。

- 角度分解能: 実車映像を AWSIM より粗い 0.250 deg/px まで落としても車線確率 0.15 を維持する。
  AWSIM は 0.234 deg/px で 0.020。同じ粗さで 6〜13 倍の差が残る
- カメラ内部パラメータの解釈: AWSIM 申告の非正方画素（fx=192 / fy=227.4）でも正方画素でも同一
- 取り付け角: 自己キャリブレーションの推定値は pitch -0.01 deg / yaw -0.62 deg でほぼゼロ
- カメラ解像度: AWSIM のビルドに焼き込まれており CLI では変更できない（上記のとおり上げても効かない）

したがって限界要因はシーンの内容であり、カート用バリアと路面標示の無いコースを
このモデルが道路として認識していない。改善にはファインチューニングかモデル差し替えを要する。

## キャリブレーション

openpilot `calibrationd` の移植。直進中はモデルが出す自車並進 `pose[:3]` の向きが
カメラ光軸と一致するはずで、そのズレを取り付け誤差とする。

```
observed = [0, -atan2(trans_z, trans_x), atan2(trans_y, trans_x)]
```

他のコントローラに運転させながら観測のみ行える（`OPENPILOT_OBSERVE=true`）。
結果は `camera.calibration_file` に保存し次回起動時に読む。

**citycircuit では上流の採用条件を満たさない。** MPC 運転時の実測（120 フレーム）:

| ゲート | 通過率 |
|---|---|
| `v_ego > 3.0` | 33 % |
| 予測 `trans_x > 3.0` | 40 % |
| ヨーレート < 2 deg/s | 55 %（ほぼ停止中のフレーム） |
| 3 つ同時 | 0 % |

走行中のヨーレート中央値が 24.7 deg/s で、openpilot が前提とする直進区間が存在しない。
低速で直進させれば「straight」の通過率は 87〜90 % となり、`calibration_min_speed` を
0.8 程度まで下げると 1 分程度で収束する。収束状況は 20 秒ごとにログへ出る。

## 実測

### 推論

| 実行環境 | 1 推論 | パイプライン全体 |
|---|---|---|
| CPU | 20〜35 ms | — |
| GPU (NVIDIA L4) | 10〜18 ms | 9.5 frames/s |

デバッグ画像の生成は推論より重く（ワープ 2 回 + 描画 + 1.5 MB メッセージ）、推論と同じスレッドを使う。
毎フレーム publish すると 9.5 → 2.3 frames/s まで低下し、積む 2 フレームの間隔が 435 ms になる。
`output.debug_image_decimation` の既定は 5。

### 走行

`direct_action` / 90〜120 秒で 22〜87 m、停止状態が時間の 9 割近く。走行ごとのばらつきが大きい。
発進アシストを恒久的な速度下限にすると 155〜174 m まで伸びるが、それは `launch_speed` で
押しているだけでモデルの走行ではない。既定は発進後に引き渡す挙動とする。

## 既知の落とし穴

- **AWSIM は `longitudinal.speed` を追従する。** 加速度のみでは動かない
- **モデルは停止中 `shouldStop` を返す。** 目標速度が 0 のままとなり自力発進しない。
  実車では人間がアクセルを踏むためモデルの担当外。`control.launch_speed` がこれを破る
- **`--wall-recovery off` では壁接触後に永久固着する。** 「制御が効かない」と区別できなくなるため、
  立ち上げ用の `simulator_scripts/openpilot.sh` は復帰を有効にしている
- `desire` 入力は常にゼロ（車線変更プランナは無い）
