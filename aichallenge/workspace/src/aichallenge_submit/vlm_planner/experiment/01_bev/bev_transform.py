import cv2
import numpy as np

# ==== カメラパラメータ (ROSのCameraInfoから) ====
fx, fy = 960.0, 959.3908
cx, cy = 960.5, 540.5
width, height = 1920, 1080

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]])

# ==== カメラ姿勢 ====
pitch_deg = 0      # カメラの下向き角度 [deg]
cam_height = 0.5    # カメラの地上高 [m]
pitch = np.deg2rad(pitch_deg)

# ==== 入力画像 ====
image = cv2.imread("original_image.png")
if image is None:
    raise FileNotFoundError("original_image.png が見つかりません")

h, w = image.shape[:2]
print(f"Loaded image {w}x{h}")

# ------------------------------------------------------------
# 🧭 パターン①：幾何的BEV変換（カメラ姿勢＋内部行列）
# ------------------------------------------------------------
# X軸回転行列（ピッチ角）
R_x = np.array([[1, 0, 0],
                [0, np.cos(pitch), -np.sin(pitch)],
                [0, np.sin(pitch),  np.cos(pitch)]])

# 地面 z=0 平面への射影変換行列
H_geom = K @ np.hstack((R_x[:, :2], np.array([[0], [cam_height], [0]])))

bev_geom = cv2.warpPerspective(image, H_geom, (w, h))

cv2.imwrite("bev_geom.jpg", bev_geom)
print("✅ 幾何的BEV変換結果: bev_geom.jpg")

# ------------------------------------------------------------
# 🧩 パターン②：擬似BEV変換（手動4点で透視補正）
# ------------------------------------------------------------
src_pts = np.float32([
    [w*0.05, h*0.60],  # 左上
    [w*0.95, h*0.60],  # 右上
    [w*0.99, h*0.75],  # 左下
    [w*0.01, h*0.75],  # 左下
])
dst_pts = np.float32([
    [w*0.25, 0],
    [w*0.75, 0],
    [w*0.75, h*0.9],
    [w*0.25, h*0.9],
])

# 透視変換行列を計算
H_pseudo = cv2.getPerspectiveTransform(src_pts, dst_pts)
bev_pseudo = cv2.warpPerspective(image, H_pseudo, (w, h))

# 元画像に4点を描画
vis = image.copy()
for i, pt in enumerate(src_pts):
    x, y = int(pt[0]), int(pt[1])
    cv2.circle(vis, (x, y), 10, (0, 0, 255), -1)
    cv2.putText(vis, f"P{i+1}", (x+10, y-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
cv2.polylines(vis, [src_pts.reshape((-1, 1, 2)).astype(int)],
              True, (0, 255, 255), 2)

cv2.imwrite("front_with_points.jpg", vis)
cv2.imwrite("bev_pseudo.jpg", bev_pseudo)
print("✅ 擬似BEV変換結果: bev_pseudo.jpg (front_with_points.jpgに領域描画あり)")
