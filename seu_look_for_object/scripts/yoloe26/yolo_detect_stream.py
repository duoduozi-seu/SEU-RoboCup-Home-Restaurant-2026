from pathlib import Path
from typing import Dict, Union

import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor


ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "yoloe-26x-seg.pt"
PROMPT_IMAGE_PATH = Path("/home/duo/yoloe26/image_prompt.JPG")
PROMPT_BBOX = [1054.00, 1695.00, 1460, 2734.00]  # Example: [221.52, 405.80, 344.98, 857.54]
VIDEO_SOURCE = 0  # Webcam index, local video path, or RTSP/HTTP stream URL
CONF = 0.15
WINDOW_NAME = "YOLOE Prompt Detection"


def load_visual_prompts(prompt_image_path: Path) -> Dict[str, np.ndarray]:
    prompt_image = cv2.imread(str(prompt_image_path))
    if prompt_image is None:
        raise RuntimeError(f"Failed to read prompt image: {prompt_image_path}")

    height, width = prompt_image.shape[:2]
    bbox = PROMPT_BBOX or [0.0, 0.0, float(width), float(height)]
    return {
        "bboxes": np.array([bbox], dtype=np.float32),
        "cls": np.array([0], dtype=np.int32),
    }


def open_video_capture(source: Union[int, str]) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise RuntimeError(f"Failed to open video source: {source}")
    return capture


def main() -> None:
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Model not found: {MODEL_PATH}")
    if not PROMPT_IMAGE_PATH.exists():
        raise FileNotFoundError(f"Prompt image not found: {PROMPT_IMAGE_PATH}")

    model = YOLO(str(MODEL_PATH))
    visual_prompts = load_visual_prompts(PROMPT_IMAGE_PATH)
    capture = open_video_capture(VIDEO_SOURCE)

    print("Press Q or Esc in the preview window to stop.")

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print("Video stream ended or frame read failed.")
                break

            results = model.predict(
                source=frame,
                conf=CONF,
                verbose=False,
                refer_image=str(PROMPT_IMAGE_PATH),
                visual_prompts=visual_prompts,
                predictor=YOLOEVPSegPredictor,
            )
            result = results[0]
            count = 0 if result.boxes is None else len(result.boxes)
            annotated = result.plot()
            cv2.putText(
                annotated,
                f"detections: {count}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, annotated)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
