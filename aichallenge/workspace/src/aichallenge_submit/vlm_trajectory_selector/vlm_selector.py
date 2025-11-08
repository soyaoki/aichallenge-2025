# vlm_selector.py

import google.generativeai as genai
import os
import time
import json
from PIL import Image
from enum import Enum
import typing_extensions as typing

from dotenv import load_dotenv

load_dotenv(verbose=True)

class VLMCommand(Enum):
    TURN_LEFT = "L"
    TURN_RIGHT = "R"
    GO_STRAIGHT = "S"
    NONE = "N"

# Add current sector to VLM response
class VlmResponse(typing.TypedDict):
    action: str
    reason: str
    current_sector: int

class ControlCommand(typing.TypedDict):
    """Response schema passed to Gemini"""
    command: str
    reason: str

class VLMSelector:
    """
    Class for image recognition and decision making using VLM
    """
    def __init__(self, logger):
        self.logger = logger
        self.model = None
        self._setup_gemini()
        self.track_knowledge = """Track sectors:
                                1: Starting Straight. At the end of the straight in sector 1, there is sector 2: a right-hand hairpin. In sector 2, there is a white sign. When you see the white sign, you must turn the steering wheel all the way to the right.
                                2: R-Hairpin (Hint: Turn hard R at the white big sign board)
                                3: Short Straight
                                4: L-Hairpin
                                5: Short Straight
                                6: U-shaped Right
                                7: 90-degree Left
                                8: R-Hairpin
                                9: L-Hairpin
                                10: S-Curve (L->R)
                                11: 90-degree Right
                                12: S-Curve (L->R)
                                13: Sweeping Right Corner (leads to main straight)"""

    def _setup_gemini(self):
        """
        Set up the Gemini model.
        """
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            self.logger.error("Environment variable 'GEMINI_API_KEY' is not set.")
            raise ValueError("API key is missing.")
        genai.configure(api_key=api_key)
        model_name = os.getenv("GEMINI_MODEL_NAME", "gemini-2.5-flash-lite")
        self.logger.info(f"""VLMSelector: VLM Model is "{model_name}".""")
        self.model = genai.GenerativeModel(model_name=model_name)
        self.logger.info("VLMSelector: Gemini model initialized successfully.")

    def _preprocess_image(self, image: Image) -> Image:
        """
        Crop and resize the image.
        """
        pil_image = image.copy()
        # Adjust recognition area
        w, h = pil_image.size
        crop_box = (0, h // 2 - 200, w, h // 2 + 200)
        pil_image = pil_image.crop(crop_box)
        pil_image = pil_image.resize((w // 4, (crop_box[3] - crop_box[1]) // 4))
        return pil_image

    def infer(self, image: Image, last_action: VLMCommand, last_sector: int) -> VLMCommand:
        """
        Infer the direction to proceed using VLM based on the given image.
        """
        if self.model is None:
            self.logger.warn("VLM model is not ready.")
            return VLMCommand.NONE

        processed_image = self._preprocess_image(image)
        
        prompt = f"""
        Based on the given image of a race track, choose the best action to drive the car.
        The road is the dark gray area. Stay on the road.
        Actions: "L"(Left), "R"(Right), "S"(Straight).
        {self.track_knowledge}
        History:
        - Last Action: {last_action.value}
        - Last sector: {last_sector}
        Your task: Determine the current sector and next action.
        """
        
        try:
            start = time.perf_counter()
            response = self.model.generate_content(
                [prompt, processed_image],
                generation_config=genai.GenerationConfig(
                    response_mime_type="application/json",
                    response_schema=VlmResponse
                ),
            )
            end = time.perf_counter()
            latency = end - start
            self.logger.info(f"Gemini latency: {latency:.4f}s")

            response_dict = json.loads(response.text.strip())
            action_str = response_dict.get("action")
            current_sector = response_dict.get("current_sector", last_sector)

            # Convert string to Enum and return
            action_map = {
                "L": VLMCommand.TURN_LEFT,
                "R": VLMCommand.TURN_RIGHT,
                "S": VLMCommand.GO_STRAIGHT,
            }
            new_action = action_map.get(action_str, VLMCommand.NONE)
            return new_action, current_sector
                
        except Exception as e:
            self.logger.error(f"Error during VLM inference: {e}")
            return VLMCommand.NONE
