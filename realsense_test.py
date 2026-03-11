#!/usr/bin/env python3
import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


# D435i（本机 `rs-enumerate-devices -v`，librealsense 2.56.5）在 BGR8 + Z16 下，
# 原生“同分辨率”可直接对齐的常用组合如下；默认优先选最高的 1280x720@30。
# 1280x720 @ 30 / 15 / 6
# 848x480  @ 60 / 30 / 15 / 6
# 640x480  @ 60 / 30 / 15 / 6
# 640x360  @ 60 / 30 / 15 / 6
# 424x240  @ 60 / 30 / 15 / 6
#
# 下面这些也可以做“对齐到彩色图”，但属于跨分辨率组合，深度会被重采样到彩色分辨率：
# color 1920x1080 + depth 1280x720 @ 30 / 15 / 6
# color 1280x720  + depth 848x480  @ 30 / 15 / 6
# color 960x540   + depth 848x480  @ 60 / 30 / 15 / 6
# color 848x480   + depth 640x480  @ 60 / 30 / 15 / 6
#
# 官方资料也建议：做 depth-to-color 对齐时，彩色流分辨率应与深度流相同或更高，
# 并尽量保持两路流使用相同帧率。

RESOLUTION_OPTIONS = [
    (1280, 720),
    (848, 480),
    (640, 480),
    (640, 360),
    (424, 240),
]
DEFAULT_RESOLUTION = max(RESOLUTION_OPTIONS, key=lambda item: item[0] * item[1])

ALIGNED_PROFILE_OPTIONS = [
    {"color": (1280, 720), "depth": (1280, 720), "fps": 30},
    {"color": (1280, 720), "depth": (1280, 720), "fps": 15},
    {"color": (1280, 720), "depth": (1280, 720), "fps": 6},
    {"color": (848, 480), "depth": (848, 480), "fps": 60},
    {"color": (848, 480), "depth": (848, 480), "fps": 30},
    {"color": (848, 480), "depth": (848, 480), "fps": 15},
    {"color": (848, 480), "depth": (848, 480), "fps": 6},
    {"color": (640, 480), "depth": (640, 480), "fps": 60},
    {"color": (640, 480), "depth": (640, 480), "fps": 30},
    {"color": (640, 480), "depth": (640, 480), "fps": 15},
    {"color": (640, 480), "depth": (640, 480), "fps": 6},
    {"color": (640, 360), "depth": (640, 360), "fps": 60},
    {"color": (640, 360), "depth": (640, 360), "fps": 30},
    {"color": (640, 360), "depth": (640, 360), "fps": 15},
    {"color": (640, 360), "depth": (640, 360), "fps": 6},
    {"color": (424, 240), "depth": (424, 240), "fps": 60},
    {"color": (424, 240), "depth": (424, 240), "fps": 30},
    {"color": (424, 240), "depth": (424, 240), "fps": 15},
    {"color": (424, 240), "depth": (424, 240), "fps": 6},
]
DEFAULT_PROFILE = next(
    profile
    for profile in ALIGNED_PROFILE_OPTIONS
    if profile["color"] == DEFAULT_RESOLUTION and profile["fps"] == 30
)
WINDOW_NAME = "RealSense D435i Aligned RGB / Depth"
DEPTH_VISUAL_ALPHA = 0.03


def color_frame_to_bgr(color_frame):
    if color_frame is None:
        return None
    return np.asanyarray(color_frame.get_data())


def depth_frame_to_array(depth_frame):
    if depth_frame is None:
        return None
    return np.asanyarray(depth_frame.get_data())


def depth_image_to_colormap(depth_image):
    if depth_image is None:
        return None
    depth_8bit = cv2.convertScaleAbs(depth_image, alpha=DEPTH_VISUAL_ALPHA)
    return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)


def build_stream_config(rs_module, profile):
    config = rs_module.config()
    depth_width, depth_height = profile["depth"]
    color_width, color_height = profile["color"]
    fps = profile["fps"]
    config.enable_stream(rs_module.stream.depth, depth_width, depth_height, rs_module.format.z16, fps)
    config.enable_stream(rs_module.stream.color, color_width, color_height, rs_module.format.bgr8, fps)
    return config


def format_profile(profile):
    color_width, color_height = profile["color"]
    depth_width, depth_height = profile["depth"]
    fps = profile["fps"]
    return f"RGB {color_width}x{color_height} + Depth {depth_width}x{depth_height} @ {fps}"


def resize_depth_preview_if_needed(color_image, depth_preview):
    if color_image is None or depth_preview is None:
        return color_image, depth_preview
    if color_image.shape[:2] == depth_preview.shape[:2]:
        return color_image, depth_preview
    resized_depth_preview = cv2.resize(depth_preview, (color_image.shape[1], color_image.shape[0]))
    return color_image, resized_depth_preview


def print_profile_catalog():
    print("内置原生同分辨率对齐档位：")
    for index, profile in enumerate(ALIGNED_PROFILE_OPTIONS):
        marker = " (default)" if profile == DEFAULT_PROFILE else ""
        print(f"  [{index:02d}] {format_profile(profile)}{marker}")


def main():
    if rs is None:
        raise RuntimeError("pyrealsense2 未安装，无法打开 RealSense 视频流。")

    pipeline = rs.pipeline()
    config = build_stream_config(rs, DEFAULT_PROFILE)
    align = rs.align(rs.stream.color)

    print_profile_catalog()
    print(f"当前启用：{format_profile(DEFAULT_PROFILE)}")

    pipeline.start(config)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            color_image = color_frame_to_bgr(color_frame)
            depth_image = depth_frame_to_array(depth_frame)
            depth_preview = depth_image_to_colormap(depth_image)
            if color_image is None or depth_image is None or depth_preview is None:
                continue

            color_image, depth_preview = resize_depth_preview_if_needed(color_image, depth_preview)
            preview = np.hstack((color_image, depth_preview))

            center_x = color_image.shape[1] // 2
            center_y = color_image.shape[0] // 2
            center_distance = depth_frame.get_distance(center_x, center_y)
            preview_text = f"{format_profile(DEFAULT_PROFILE)} | center depth {center_distance:.3f}m | q/ESC exit"
            cv2.putText(
                preview,
                preview_text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow(WINDOW_NAME, preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
