from pathlib import Path
from typing import Tuple

import cv2


IMAGE_PATH = Path("/home/duo/yoloe26/image_prompt.JPG")
WINDOW_NAME = "Select Prompt Box"
BOX_COMMENT = "Box enclosing target"
DECIMALS = 2
SCREEN_MARGIN = 120
DEFAULT_SCREEN_SIZE = (1600, 900)


def format_box(x: int, y: int, w: int, h: int) -> str:
    x1 = round(float(x), DECIMALS)
    y1 = round(float(y), DECIMALS)
    x2 = round(float(x + w), DECIMALS)
    y2 = round(float(y + h), DECIMALS)
    return f"[{x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f}],  # {BOX_COMMENT}"


def get_screen_size() -> Tuple[int, int]:
    try:
        import tkinter as tk

        root = tk.Tk()
        root.withdraw()
        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()
        root.destroy()
        return width, height
    except Exception:
        return DEFAULT_SCREEN_SIZE


def resize_for_display(image):
    image_height, image_width = image.shape[:2]
    screen_width, screen_height = get_screen_size()
    max_width = max(1, screen_width - SCREEN_MARGIN)
    max_height = max(1, screen_height - SCREEN_MARGIN)
    scale = min(max_width / image_width, max_height / image_height, 1.0)

    if scale == 1.0:
        return image, scale

    resized = cv2.resize(
        image,
        (int(image_width * scale), int(image_height * scale)),
        interpolation=cv2.INTER_AREA,
    )
    return resized, scale


def main() -> None:
    if not IMAGE_PATH.exists():
        raise FileNotFoundError(f"Image not found: {IMAGE_PATH}")

    image = cv2.imread(str(IMAGE_PATH))
    if image is None:
        raise RuntimeError(f"Failed to read image: {IMAGE_PATH}")

    display_image, scale = resize_for_display(image)
    print("Drag a box, then press Enter or Space to confirm. Press C to cancel.")
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    x, y, w, h = cv2.selectROI(WINDOW_NAME, display_image, showCrosshair=True, fromCenter=False)
    cv2.destroyAllWindows()

    if w <= 0 or h <= 0:
        print("No box selected.")
        return

    x = int(round(x / scale))
    y = int(round(y / scale))
    w = int(round(w / scale))
    h = int(round(h / scale))
    print(format_box(x, y, w, h))


if __name__ == "__main__":
    main()
