#!/usr/bin/env python3
import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


RESOLUTION_OPTIONS = [
    (640, 480),
    (1280, 720),
    (1920, 1080),
]
DEFAULT_RESOLUTION = max(RESOLUTION_OPTIONS, key=lambda item: item[0] * item[1])
WINDOW_NAME = "RealSense D435i Preview"


def color_frame_to_bgr(color_frame):
    if color_frame is None:
        return None
    return np.asanyarray(color_frame.get_data())


def main():
    if rs is None:
        raise RuntimeError("pyrealsense2 未安装，无法打开 RealSense 视频流。")

    pipeline = rs.pipeline()
    config = rs.config()

    # 分辨率选项，默认启用最大分辨率
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            image = color_frame_to_bgr(color_frame)
            if image is None:
                continue

            cv2.putText(
                image,
                f"{DEFAULT_RESOLUTION[0]}x{DEFAULT_RESOLUTION[1]}  Press q/ESC to exit",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
            )
            cv2.imshow(WINDOW_NAME, image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
