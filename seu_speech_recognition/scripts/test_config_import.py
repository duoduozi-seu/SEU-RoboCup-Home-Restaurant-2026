import config
import os

print(f"WAV_DIR: {config.WAV_DIR}")
print(f"AUDIO_OUTPUT_PATH: {config.AUDIO_OUTPUT_PATH}")
print(f"MODEL_PATH: {config.MODEL_PATH}")

if os.path.exists(config.MODEL_PATH):
    print("Model file found!")
else:
    print("Model file NOT found (this might be expected if not downloaded yet)")
