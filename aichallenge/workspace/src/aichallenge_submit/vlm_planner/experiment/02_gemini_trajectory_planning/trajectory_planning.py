# 参考：https://ai.google.dev/gemini-api/docs/robotics-overview?hl=ja#trajectories

import os
import time
from dotenv import load_dotenv
from google import genai
from google.genai import types
from pydantic import BaseModel, Field
from typing import List
from PIL import Image, ImageDraw, ImageFont

# -----------------------------------------------------------
# 1. 環境設定と初期化
# -----------------------------------------------------------

# .env ファイルから環境変数をロード
load_dotenv("../../.env", verbose=True)

# 利用可能モデルの候補
available_models = [
    "gemini-robotics-er-1.5-preview",
    "gemini-2.5-flash-lite",
]

print("Select Gemini model to use:")
for i, model_name in enumerate(available_models, start=1):
    print(f"{i}: {model_name}")

while True:
    choice = input(f"Enter number (1-{len(available_models)}): ").strip()
    if choice.isdigit() and 1 <= int(choice) <= len(available_models):
        MODEL_ID = available_models[int(choice)-1]
        break
    else:
        print("Invalid choice. Please try again.")

print(f"Selected model: {MODEL_ID}")

# Gemini Clientの初期化
client = genai.Client(api_key=os.getenv("GEMINI_API_KEY"))

# -----------------------------------------------------------
# 2. Pydantic Response Schemaの定義
# -----------------------------------------------------------

class TrajectoryPoint(BaseModel):
    """自車の軌跡上の一点 ([y, x])"""
    point: List[int] = Field(description="Coordinates of the point in [y, x]. Origin (0,0) is at the top-left corner of the image.")
    label: str = Field(description="Sequential label of the point, starting from '0' to '14'.")

class TrajectoryResponse(BaseModel):
    """自車の予測軌跡を表す15点のリストをラップするコンテナ"""
    trajectory: List[TrajectoryPoint] = Field(description="A list of exactly 15 trajectory points, labeled '0' to '14'.")

# -----------------------------------------------------------
# 3. 画像のロードとプロンプトの定義
# -----------------------------------------------------------

# 利用可能モデルの候補
available_image_filenames = [
    "original_image.png",
    "bev_pseudo.jpg",
]

print("Select image file:")
for i, image_filename in enumerate(available_image_filenames, start=1):
    print(f"{i}: {image_filename}")

while True:
    choice = input(f"Enter number (1-{len(image_filename)}): ").strip()
    if choice.isdigit() and 1 <= int(choice) <= len(available_image_filenames):
        IMAGE_FILENAME = available_image_filenames[int(choice)-1]
        break
    else:
        print("Invalid choice. Please try again.")

print(f"Selected image file: {IMAGE_FILENAME}")

OUTPUT_FILENAME = f'output_with_trajectory_points_{MODEL_ID}_{os.path.splitext(IMAGE_FILENAME)[0]}.jpg'

try:
    with open(IMAGE_FILENAME, 'rb') as f:
        image_bytes = f.read()
except FileNotFoundError:
    print(f"Error: Image file '{IMAGE_FILENAME}' not found.")
    exit()

points_data = []

# 自車の軌跡計画プロンプト
prompt = """
Place a point on lower center center of input image, then plan 15 points for the trajectory of moving to keep road center.
The points should be labeled by order of the trajectory, from '0' (lower center of the image) to <n> (final point)
The answer should follow the json format:
[{"point": <point>, "label": <label1>}, ...].
The points are in [y, x] format normalized to 0-1000.
"""

# -----------------------------------------------------------
# 4. Gemini APIの呼び出しとJSONスキーマの適用
# -----------------------------------------------------------

print(f"--- Calling Gemini API with model: {MODEL_ID} ---")

start_time = time.perf_counter()

image_response = client.models.generate_content(
  model=MODEL_ID,
  contents=[
    types.Part.from_bytes(
      data=image_bytes,
      mime_type='image/jpeg',
    ),
    prompt
  ],
  config = types.GenerateContentConfig(
      temperature=0.0,
      seed=42,
      response_mime_type="application/json",
      response_json_schema=TrajectoryResponse.model_json_schema(),
  )
)

end_time = time.perf_counter()
latency_ms = (end_time - start_time) * 1000

print("\n--- Raw Model Output (should be pure JSON) ---")
print(image_response.text)

print(f"\n--- Inference Latency ---")
print(f"Time taken for inference: **{latency_ms:.2f} ms**")

# -----------------------------------------------------------
# 5. Pydanticによる応答の検証とデータ抽出
# -----------------------------------------------------------

try:
    # JSON文字列をPydanticモデルで検証しながらパース
    trajectory_data = TrajectoryResponse.model_validate_json(image_response.text)
    
    # TrajectoryResponseオブジェクトからポイントデータを抽出
    for point_info in trajectory_data.trajectory:
        point = point_info.point  # point は [y, x] のリスト
        label = point_info.label  # label は str
        points_data.append((point, label))

    print(f"\nSuccessfully extracted {len(points_data)} trajectory points.")
    
except Exception as e:
    print(f"\n--- Pydantic Validation/Parsing Error ---")
    print(f"Error: {e}")
    # エラーが発生した場合は、後続の可視化処理をスキップ
    exit()


# -----------------------------------------------------------
# 6. 結果の可視化
# -----------------------------------------------------------

# Visualize points on the image
image = Image.open(IMAGE_FILENAME).convert("RGB") # RGBに変換してアルファチャンネルを扱う準備
draw = ImageDraw.Draw(image)

# フォントの設定（必要であれば、カスタムフォントをロードすることも可能）
try:
    # 存在すればRobotoなどを利用、なければデフォルトフォント
    font = ImageFont.truetype("arial.ttf", 20) 
except IOError:
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    except IOError:
        font = ImageFont.load_default()
        print("Warning: TrueType font not found, using default font (サイズ変更不可)")


# 描画設定
POINT_RADIUS = 10
FILL_COLOR = (66, 133, 244, 200)
OUTLINE_COLOR = (255, 255, 255, 255)
LABEL_COLOR = (66, 133, 244, 255)

# 軌跡の線を引くための準備
previous_point_coords = None

for i, (point, label) in enumerate(points_data):
    # 座標を正規化された 0-1000 からピクセル値に変換
    y = int(point[0] / 1000 * image.height)
    x = int(point[1] / 1000 * image.width)
    
    current_point_coords = (x, y)

    # ポイントを描画
    draw.ellipse((x - POINT_RADIUS, y - POINT_RADIUS, x + POINT_RADIUS, y + POINT_RADIUS),
                 fill=OUTLINE_COLOR) # 外側の白枠
    draw.ellipse((x - POINT_RADIUS + 2, y - POINT_RADIUS + 2, x + POINT_RADIUS - 2, y + POINT_RADIUS - 2),
                 fill=FILL_COLOR) # 内側の青い塗りつぶし

    # ラベルを描画
    draw.text((x + POINT_RADIUS + 5, y - 10), label, fill=LABEL_COLOR, font=font, size=20)

    # ポイント間に線を引く
    if previous_point_coords:
        draw.line([previous_point_coords, current_point_coords], fill=FILL_COLOR, width=3)
    
    previous_point_coords = current_point_coords

image.save(OUTPUT_FILENAME) 
print(f"\n--- Success ---")
print(f"Trajectory points visualized and saved to {OUTPUT_FILENAME}")
