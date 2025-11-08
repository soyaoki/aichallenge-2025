from transformers import AutoConfig, AutoModelForImageTextToText, AutoProcessor

MODEL_NAME = "turing-motors/Heron-NVILA-Lite-1B-hf"
# MODEL_NAME = "turing-motors/Heron-NVILA-Lite-2B-hf"
# MODEL_NAME = "turing-motors/Heron-NVILA-Lite-15B-hf"

# you can use config
config = AutoConfig.from_pretrained(MODEL_NAME, trust_remote_code=True)
model = AutoModelForImageTextToText.from_config(config, trust_remote_code=True)

# or directly from_pretrained
model = AutoModelForImageTextToText.from_pretrained(MODEL_NAME, trust_remote_code=True, device_map="auto")

# load processor
processor = AutoProcessor.from_pretrained(MODEL_NAME, trust_remote_code=True)

# show chat_template
print(processor.tokenizer.chat_template)

def generate_content(content: str, images: list | None = None, **kwargs) -> str:
    conversation = [{"role": "user", "content": content}]
    text = processor.apply_chat_template(conversation, add_generation_prompt=True, tokenize=False)
    encoding = processor(text=text, images=images, return_tensors="pt").to(model.device)
    output = model.generate(**encoding, **kwargs)
    return processor.decode(output[0, len(encoding["input_ids"][0]):], skip_special_tokens=True)

# examples generate with text + image
from PIL import Image
image = Image.open("original_image.png").convert("RGB")
response = generate_content("<image>\n画像を説明してください。", images=[image])
print(response)
print("---" * 40)
