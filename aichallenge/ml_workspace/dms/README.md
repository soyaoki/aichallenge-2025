# DMS ワークスペース

openpilot のドライバー監視モデル `dmonitoring_model.onnx` を、シミュレータや実車を
使わずに手元の動画で試すための実験用ワークスペース。競技の提出物ではないので
ROS ノードにはしていない。

走行モデル側の統合は
[openpilot_controller](../../workspace/src/aichallenge_submit/openpilot_controller) を参照。

## 使い方

```bash
./download_model.bash                       # 重みを取得して SHA-256 を検証
python3 run_dms.py --video clip.mp4
python3 run_dms.py --video clip.mp4 --hfov-deg 60 --annotate out.mp4
```

`face_prob` / 両目の検出 / まばたき / サングラス / スマホ使用 / 眠気 を左右ハンドル別に
集計する。左右は平均 `face_prob` が高い方を自動で選ぶ。

## 画角の指定が効く

openpilot はドライバーカメラを 1440x960 の輝度 1 面へ切り出してモデルへ渡す。
`dmonitoringmodel_intrinsics` の焦点距離はカメラと同じ 567 なので、この変換は
拡大縮小のない**ただのクロップ**であり、魚眼の歪み補正はしていない。

別のカメラの映像を使う場合は同じ角度スケールに合わせる必要があり、それが
`--hfov-deg` にあたる。合成動画 (1280x720) での実測:

| 仮定画角 | `face_prob` |
|---|---|
| 40° | 0.671 |
| 60° | 0.803 |
| **78°** | **0.844** |
| 104°（モデル座標系と同じ） | 0.309 |
| 140° | 0.169 |

最良は 78° で、openpilot が webcam 運用で推奨していた画角と一致する。
**魚眼として扱うと逆に落ちる。**

## `face_prob` が組み込みの判定器になる

走行モデルでは「車線が出ない」のが前処理のバグなのか入力がドメイン外なのかを
外部データ無しでは切り分けられなかった。DMS は `face_prob` が高ければ
「顔を見つけた＝前処理は当たっている」と即座に分かるので、その心配がない。
`face_prob` が低いまま `sleep_prob` を読んでも意味がないため、スクリプトは
0.3 を下回ったら警告する。

## 実測メモ

[Cosmos で生成した居眠りシーン](https://qiita.com/soyaoki/items/ee8b5e73512e3c2d7272)
（1280x720 / 24fps / 2秒）に対して hfov 78° で:

| 信号 | 平均 | 最大 |
|---|---|---|
| `face_prob` | 0.844 | 0.943 |
| `left/right_eye_prob` | 0.89 / 0.85 | 0.96 / 0.94 |
| `left/right_blink_prob` | 0.46 / 0.47 | 0.79 / 0.82 |
| `using_phone_prob` | 0.026 | 0.035 |
| `sleep_prob` | 0.004 | 0.011 |

まばたき確率が閉眼の動きに追従する（前半 0.10〜0.28、後半 0.74〜0.80）。
**合成データでも顔と閉眼を拾える。**

`sleep_prob` はほとんど動かない。openpilot 本体の眠気判定は
`selfdrive/monitoring/` 側でまばたきを時間方向に積分する構造なので、
居眠りの評価には `blink_prob` の持続時間を見るほうが妥当と思われる。

## 重みのバージョン

master の `dmonitoring_model.onnx` は Git LFS オブジェクトがサーバ側に存在せず
取得できない（`Object does not exist on the server`）。取得できる最新である
v0.11.1 に固定している。入力仕様は v0.9.7 から変わっていない
（`input_img (1,1382400)` + `calib (1,3)`、dtype が float32 から uint8 になっただけ）。
