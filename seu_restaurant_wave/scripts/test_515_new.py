#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
# -*- coding: utf-8 -*-
"""Standalone visual test for the current restaurant wave distance logic.

This script intentionally avoids rospy.  It reuses the core detector methods in
wave_detector_core.py, while owning the RealSense and OpenCV visualization loop
locally so it can be run as a plain Python program.
"""

import argparse
import ast
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

import cv2
import numpy as np
import torch
from ultralytics import YOLO


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from wave_detector_core import (  # noqa: E402
    DEPTH_FRAME_HEIGHT,
    DEPTH_FRAME_WIDTH,
    FRAME_FPS,
    FRAME_HEIGHT,
    FRAME_WIDTH,
    KEYPOINT_VISIBILITY,
    POSE_MODEL_PATH,
    WAVE_CAMERA_SERIAL_NO,
    WaveDetectorRuntime,
    select_nearest_waving_candidate,
)


WINDOW_NAME = "restaurant wave depth visual test"
DEPTH_VISUAL_ALPHA = 0.03
DEFAULT_CAMERA_BACKWARD_BIAS_M = -0.85

BOX_COLOR_NORMAL = (160, 160, 160)
BOX_COLOR_WAVING = (0, 255, 0)
POINT_COLOR_BOX_CENTER = (0, 0, 255)
POINT_COLOR_SHOULDER_CENTER = (0, 255, 0)
TEXT_COLOR = (255, 255, 255)
TEXT_BG_COLOR = (0, 0, 0)


def load_camera_backward_bias():
    """Read CAMERA_BACKWARD_BIAS_M without importing the ROS action server."""
    server_path = SCRIPT_DIR / "find_waving_customer_action_server.py"
    try:
        tree = ast.parse(server_path.read_text(encoding="utf-8"))
        for node in tree.body:
            if not isinstance(node, ast.Assign):
                continue
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "CAMERA_BACKWARD_BIAS_M":
                    return float(ast.literal_eval(node.value))
    except Exception:
        pass
    return DEFAULT_CAMERA_BACKWARD_BIAS_M


CAMERA_BACKWARD_BIAS_M = load_camera_backward_bias()


class StandaloneWaveDetector(WaveDetectorRuntime):
    """WaveDetectorRuntime without rospy dependency in __init__."""

    def __init__(self, model_path, camera_serial=None, force_cpu=False):
        import pyrealsense2 as rs

        self.rs = rs
        self.pose_device = "cpu" if force_cpu else (0 if torch.cuda.is_available() else "cpu")
        if not os.path.exists(model_path):
            raise FileNotFoundError("wave model not found: {}".format(model_path))

        self.pose_model = YOLO(model_path)
        self.pipeline = rs.pipeline()
        config = rs.config()
        if camera_serial:
            config.enable_device(camera_serial)
        config.enable_stream(rs.stream.depth, DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT, rs.format.z16, FRAME_FPS)
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FRAME_FPS)

        self.align_to_color = rs.align(rs.stream.color)
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling_filter = rs.hole_filling_filter()
        self.profile = self.pipeline.start(config)

        device = self.profile.get_device()
        actual_sn = device.get_info(rs.camera_info.serial_number)
        if camera_serial and actual_sn != camera_serial:
            raise RuntimeError(
                "Connected wrong RealSense device for wave: expected {}, got {}".format(
                    camera_serial,
                    actual_sn,
                )
            )
        print("[restaurant_wave][CAM] Connected RealSense SN: {}".format(actual_sn))

        self.frame_index = 0
        self.next_track_id = 1
        self.tracks = {}

    def process_once_for_visualization(self):
        """Run one frame with the same extraction/tracking/depth path as core."""
        self.frame_index += 1

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align_to_color.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None

        depth_frame = self.spatial_filter.process(depth_frame).as_depth_frame()
        depth_frame = self.temporal_filter.process(depth_frame).as_depth_frame()
        depth_frame = self.hole_filling_filter.process(depth_frame).as_depth_frame()

        color_image = self._frame_to_numpy(color_frame)
        depth_image = np.asanyarray(depth_frame.get_data())
        detections = self._extract_detections(color_image, depth_frame)
        current_track_ids = self._update_tracks(detections)

        current_candidates = [self._clone_track_candidate(track_id) for track_id in current_track_ids]
        current_candidates = self._attach_camera_points_to_candidates(depth_frame, current_candidates)
        selected_candidate = select_nearest_waving_candidate(current_candidates)

        return {
            "color_image": color_image,
            "depth_image": depth_image,
            "current_candidates": current_candidates,
            "selected_candidate": selected_candidate,
        }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plain-Python visualizer for restaurant wave depth calculation."
    )
    parser.add_argument("--model-path", default=POSE_MODEL_PATH, help="YOLO pose model path.")
    parser.add_argument(
        "--serial",
        default=WAVE_CAMERA_SERIAL_NO,
        help="RealSense serial number. Use --any-camera to skip this.",
    )
    parser.add_argument("--any-camera", action="store_true", help="Do not force a RealSense serial number.")
    parser.add_argument("--cpu", action="store_true", help="Force YOLO inference on CPU.")
    parser.add_argument("--no-depth-panel", action="store_true", help="Only show the color panel.")
    return parser.parse_args()


def visible_point(point):
    return point is not None and float(point[2]) > KEYPOINT_VISIBILITY


def get_shoulder_center(point_map):
    left = point_map.get("left_shoulder")
    right = point_map.get("right_shoulder")
    if not (visible_point(left) and visible_point(right)):
        return None
    center_x = int(round((float(left[0]) + float(right[0])) / 2.0))
    center_y = int(round((float(left[1]) + float(right[1])) / 2.0))
    return center_x, center_y


def get_biased_distance_m(candidate):
    """Mirror camera_point_to_base_link_point(): base x = optical z + bias."""
    depth_m = float(candidate.get("depth_m", 0.0))
    if depth_m <= 0.0:
        return None
    camera_point = candidate.get("camera_point")
    if camera_point is not None:
        return float(camera_point[2]) + CAMERA_BACKWARD_BIAS_M
    return depth_m + CAMERA_BACKWARD_BIAS_M


def draw_text(image, text, origin, color=TEXT_COLOR, scale=0.55, thickness=1):
    x, y = int(origin[0]), int(origin[1])
    font = cv2.FONT_HERSHEY_SIMPLEX
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x2 = min(image.shape[1] - 1, x + text_width + 8)
    y1 = max(0, y - text_height - baseline - 6)
    y2 = min(image.shape[0] - 1, y + baseline + 4)
    cv2.rectangle(image, (max(0, x - 4), y1), (x2, y2), TEXT_BG_COLOR, -1)
    cv2.putText(image, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def draw_depth_and_shoulder_points(image, candidate):
    depth_pixel = candidate.get("depth_pixel")
    if depth_pixel is not None:
        center_x, center_y = depth_pixel
        cv2.circle(image, (int(center_x), int(center_y)), 6, POINT_COLOR_BOX_CENTER, -1)
        cv2.circle(image, (int(center_x), int(center_y)), 7, (255, 255, 255), 1)

    shoulder_center = get_shoulder_center(candidate.get("point_map", {}))
    if shoulder_center is not None:
        cv2.circle(image, shoulder_center, 6, POINT_COLOR_SHOULDER_CENTER, -1)
        cv2.circle(image, shoulder_center, 7, (255, 255, 255), 1)


def build_depth_preview(depth_image, target_shape):
    depth_preview = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=DEPTH_VISUAL_ALPHA),
        cv2.COLORMAP_JET,
    )
    if depth_preview.shape[:2] != target_shape[:2]:
        depth_preview = cv2.resize(depth_preview, (target_shape[1], target_shape[0]))
    return depth_preview


def format_distance_label(candidate):
    raw_depth_m = float(candidate.get("depth_m", 0.0))
    biased_depth_m = get_biased_distance_m(candidate)
    if raw_depth_m <= 0.0 or biased_depth_m is None:
        return "no_bias -- | with_bias --"
    return "no_bias {:.2f}m | with_bias {:.2f}m".format(raw_depth_m, biased_depth_m)


def draw_candidate(image, candidate, selected_candidate):
    box = candidate.get("box")
    if box is None:
        return

    x1, y1, x2, y2 = [int(value) for value in box]
    is_waving = bool(candidate.get("is_waving", False))
    is_selected = candidate is selected_candidate
    box_color = BOX_COLOR_WAVING if is_waving else BOX_COLOR_NORMAL
    thickness = 3 if is_selected else (2 if is_waving else 1)

    cv2.rectangle(image, (x1, y1), (x2, y2), box_color, thickness)
    draw_depth_and_shoulder_points(image, candidate)

    track_id = candidate.get("track_id", "-")
    status = "waving" if is_waving else "person"
    if is_selected:
        status = "nearest_waving"
    label_top = "ID {} {}".format(track_id, status)
    label_bottom = format_distance_label(candidate)
    label_top_y = y1 - 24
    label_bottom_y = y1 - 4
    if label_top_y < 18:
        label_top_y = min(image.shape[0] - 28, y2 + 18)
        label_bottom_y = min(image.shape[0] - 8, label_top_y + 20)
    draw_text(image, label_top, (x1, label_top_y), color=box_color, scale=0.55, thickness=1)
    draw_text(image, label_bottom, (x1, label_bottom_y), color=box_color, scale=0.55, thickness=1)


def render_visualization(frame_result, show_depth_panel=True, fps=0.0):
    color_image = frame_result["color_image"].copy()
    depth_preview = build_depth_preview(frame_result["depth_image"], color_image.shape)
    candidates = frame_result.get("current_candidates") or []
    selected_candidate = frame_result.get("selected_candidate")

    for candidate in candidates:
        draw_candidate(color_image, candidate, selected_candidate)
        draw_candidate(depth_preview, candidate, selected_candidate)

    waving_count = sum(1 for candidate in candidates if candidate.get("is_waving", False))
    selected_id = "-"
    if selected_candidate is not None:
        selected_id = selected_candidate.get("track_id", "-")

    status_text = (
        "persons {} | waving {} | selected {} | bias {:+.2f}m | fps {:.1f} | q/ESC exit".format(
            len(candidates),
            waving_count,
            selected_id,
            CAMERA_BACKWARD_BIAS_M,
            fps,
        )
    )
    draw_text(color_image, status_text, (20, 30), color=(0, 255, 0), scale=0.65, thickness=2)
    draw_text(color_image, "red: box center/depth sample | green: shoulder center", (20, 62), scale=0.55)

    if not show_depth_panel:
        return color_image
    return np.hstack((color_image, depth_preview))


def main():
    args = parse_args()
    camera_serial = None if args.any_camera else args.serial

    print("[restaurant_wave] model: {}".format(args.model_path))
    print("[restaurant_wave] depth logic: box center, 20x20 sorted 30%-60% mean, 5-frame median")
    print("[restaurant_wave] with_bias distance uses depth + CAMERA_BACKWARD_BIAS_M ({:+.2f}m)".format(
        CAMERA_BACKWARD_BIAS_M
    ))

    detector = StandaloneWaveDetector(
        model_path=args.model_path,
        camera_serial=camera_serial,
        force_cpu=args.cpu,
    )

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    last_time = time.time()

    try:
        while True:
            frame_result = detector.process_once_for_visualization()
            if frame_result is None:
                continue

            now = time.time()
            fps = 1.0 / max(1e-6, now - last_time)
            last_time = now

            preview = render_visualization(
                frame_result,
                show_depth_panel=not args.no_depth_panel,
                fps=fps,
            )
            cv2.imshow(WINDOW_NAME, preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        detector.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
