#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
# -*- coding: utf-8 -*-
"""餐厅题挥手检测核心工具。

说明：
1. 这个文件只保留“感知侧”的逻辑，不直接依赖 rospy；
2. RealSense、YOLO26、时序挥手判定都从 res_demo 迁移而来；
3. action server 只负责底盘旋转、TF 转换和 action 协议。
"""

import os
from collections import deque
from typing import Iterable, Optional


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# POSE_MODEL_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), "models", "yolo26l-pose.pt")
POSE_MODEL_PATH = '/home/duo/GPSR_ws_zyd/res_demo/yolo26m-pose.pt'

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
DEPTH_FRAME_WIDTH = 1024
DEPTH_FRAME_HEIGHT = 768
FRAME_FPS = 30
WAVE_CAMERA_SERIAL_NO = "f0244552"
POSE_IMGSZ = 1280
POSE_CONF = 0.35
POSE_IOU = 0.45
KEYPOINT_VISIBILITY = 0.35
TRACK_KEEP_FRAMES = 18
WAVE_HISTORY = 14
DEPTH_BOX_SAMPLE_LOWER_RATIO = 0.25
DEPTH_BOX_SAMPLE_UPPER_RATIO = 0.60
SIMPLE_RAISE_REQUIRED_FRAMES = 10
SIMPLE_RAISE_MARGIN_SHOULDER_WIDTH = 0.10

KEYPOINT_INDEX = {
    "left_shoulder": 5,
    "right_shoulder": 6,
    "left_elbow": 7,
    "right_elbow": 8,
    "left_wrist": 9,
    "right_wrist": 10,
    "left_hip": 11,
    "right_hip": 12,
}


def select_nearest_waving_candidate(candidates: Iterable[dict]) -> Optional[dict]:
    """从候选列表中挑出最近且深度有效的挥手目标。

    Args:
        candidates: 每个元素至少需要包含：
            - ``is_waving``: 当前人是否已被判定为挥手。
            - ``depth_m``: 当前人的稳定深度，单位米。

    Returns:
        最近的有效挥手候选；若没有满足条件的候选，则返回 ``None``。
    """
    nearest_candidates = select_nearest_waving_candidates(candidates, max_count=1)
    if not nearest_candidates:
        return None
    return nearest_candidates[0]


def select_nearest_waving_candidates(candidates: Iterable[dict], max_count: int = 2):
    """从候选列表中挑出最近的若干个有效挥手目标。"""
    valid_candidates = []
    for candidate in candidates:
        if not candidate.get("is_waving", False):
            continue
        depth_m = float(candidate.get("depth_m", 0.0))
        if depth_m <= 0.0:
            continue
        valid_candidates.append(candidate)

    valid_candidates.sort(key=lambda item: float(item["depth_m"]))
    return valid_candidates[: max(0, int(max_count))]


def average_camera_points(camera_points):
    """对多帧相机坐标取平均，用于最终目标点。"""
    valid_points = []
    for camera_point in camera_points or []:
        if camera_point is None or len(camera_point) < 3:
            continue
        valid_points.append((float(camera_point[0]), float(camera_point[1]), float(camera_point[2])))

    if not valid_points:
        return None

    point_count = float(len(valid_points))
    return (
        sum(point[0] for point in valid_points) / point_count,
        sum(point[1] for point in valid_points) / point_count,
        sum(point[2] for point in valid_points) / point_count,
    )


def clamp_box(x1, y1, x2, y2, width, height):
    """把检测框限制到图像内部。"""
    x1 = max(0, min(int(x1), width - 1))
    y1 = max(0, min(int(y1), height - 1))
    x2 = max(0, min(int(x2), width - 1))
    y2 = max(0, min(int(y2), height - 1))
    if x2 <= x1:
        x2 = min(width - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(height - 1, y1 + 1)
    return x1, y1, x2, y2


def bbox_iou(box_a, box_b):
    """使用 IoU 做最轻量的跨帧关联。"""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1, (bx2 - bx1) * (by2 - by1))
    return inter_area / float(area_a + area_b - inter_area)


def sample_box_depth_mean(
    depth_frame,
    person_box,
    lower_ratio=DEPTH_BOX_SAMPLE_LOWER_RATIO,
    upper_ratio=DEPTH_BOX_SAMPLE_UPPER_RATIO,
):
    """在整个人体框内取 20%-70% 有效深度值均值。"""
    profile = depth_frame.get_profile().as_video_stream_profile()
    width = profile.width()
    height = profile.height()
    x1, y1, x2, y2 = clamp_box(person_box[0], person_box[1], person_box[2], person_box[3], width, height)
    valid_depths = []

    for pixel_y in range(y1, y2 + 1):
        for pixel_x in range(x1, x2 + 1):
            depth_value = depth_frame.get_distance(pixel_x, pixel_y)
            if depth_value > 0.0:
                valid_depths.append(depth_value)

    if not valid_depths:
        return 0.0
    valid_depths.sort()
    sample_count = len(valid_depths)
    lower_index = max(0, min(sample_count - 1, int(sample_count * lower_ratio)))
    upper_index = max(lower_index + 1, min(sample_count, int(sample_count * upper_ratio)))
    selected_depths = valid_depths[lower_index:upper_index]
    return float(sum(selected_depths) / len(selected_depths))


def render_debug_image(color_image, candidates, hit_track_ids=None):
    """把当前候选人与最终选中目标简单画到调试图像上。

    约定：
    1. 所有当前候选人都会画框，方便看见当前帧 YOLO 检测结果；
    2. 已满足挥手条件但尚未命中的人用黄色框；
    3. 在当前离散角度里已命中的 track 用绿色框；
    4. 若没有候选人，直接返回原图副本。
    """
    import cv2

    debug_image = color_image.copy()
    hit_track_ids = set(hit_track_ids or [])

    for candidate in candidates:
        box = candidate.get("box")
        if box is None:
            continue

        x1, y1, x2, y2 = [int(value) for value in box]
        track_id = candidate.get("track_id")
        is_hit_target = track_id in hit_track_ids if track_id is not None else False
        is_waving = bool(candidate.get("is_waving", False))

        color = (160, 160, 160)
        thickness = 1
        label = "person"

        if is_waving:
            color = (0, 255, 255)
            thickness = 2
            label = "waving"
        if is_hit_target:
            color = (0, 255, 0)
            thickness = 3
            label = "target"

        depth_m = float(candidate.get("depth_m", 0.0))
        if depth_m > 0.0:
            label = "{} {:.2f}m".format(label, depth_m)

        cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(
            debug_image,
            label,
            (x1, max(15, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

        depth_pixel = candidate.get("depth_pixel")
        if depth_pixel is not None:
            center_x, center_y = depth_pixel
            cv2.circle(debug_image, (int(center_x), int(center_y)), 4, color, -1)

    return debug_image


def evaluate_simple_raise(raised_history):
    """简化挥手规则：最近若干帧里手举起次数足够多，就判为目标动作。"""
    if len(raised_history) < WAVE_HISTORY:
        return False
    return sum(1 for raised in raised_history if raised) >= SIMPLE_RAISE_REQUIRED_FRAMES


class WaveDetectorRuntime:
    """YOLO26 + RealSense 挥手检测运行时。

    这个类只提供“每处理一帧，输出当前帧候选人与深度”的能力。
    """

    def __init__(self, model_path=POSE_MODEL_PATH):
        import pyrealsense2 as rs
        import rospy
        import torch
        from ultralytics import YOLO

        self.rs = rs
        self.pose_device = 0 if torch.cuda.is_available() else "cpu"
        if not os.path.exists(model_path):
            raise FileNotFoundError("wave model not found: {}".format(model_path))

        self.pose_model = YOLO(model_path)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(WAVE_CAMERA_SERIAL_NO)

        config.enable_stream(rs.stream.depth, DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT, rs.format.z16, FRAME_FPS)
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FRAME_FPS)
        for device in rs.context().query_devices():
            if device.get_info(rs.camera_info.serial_number).lower() == WAVE_CAMERA_SERIAL_NO.lower():
                depth_sensor = device.first_depth_sensor()
                if depth_sensor.supports(rs.option.visual_preset):
                    depth_sensor.set_option(rs.option.visual_preset, int(rs.l500_visual_preset.max_range))
                break
        self.align_to_color = rs.align(rs.stream.color)
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling_filter = rs.hole_filling_filter()
        self.profile = self.pipeline.start(config)
        device = self.profile.get_device()
        actual_sn = device.get_info(rs.camera_info.serial_number)
        rospy.loginfo("[restaurant_wave][CAM] Connected RealSense SN: %s", actual_sn)
        if actual_sn != WAVE_CAMERA_SERIAL_NO:
            raise RuntimeError(
                "Connected wrong RealSense device for wave: expected {}, got {}".format(
                    WAVE_CAMERA_SERIAL_NO,
                    actual_sn,
                )
            )

        self.frame_index = 0
        self.next_track_id = 1
        self.tracks = {}

    def close(self):
        """关闭 RealSense 流，并释放本次 action 使用的 YOLO CUDA 显存。"""
        try:
            self.pipeline.stop()
        except Exception:
            pass
        try:
            if getattr(self, "pose_model", None) is not None:
                self.pose_model.to("cpu")
                self.pose_model = None
        except Exception:
            self.pose_model = None
        try:
            import gc
            import torch

            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.synchronize()
                torch.cuda.empty_cache()
        except Exception:
            pass

    def process_once(self):
        """处理一帧，并返回当前帧图像与候选人列表。"""
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
        detections = self._extract_detections(color_image, depth_frame)
        current_track_ids = self._update_tracks(detections)

        current_candidates = [self._clone_track_candidate(track_id) for track_id in current_track_ids]
        current_candidates = self._attach_camera_points_to_candidates(depth_frame, current_candidates)
        return {
            "color_image": color_image,
            "current_candidates": current_candidates,
        }

    @staticmethod
    def _frame_to_numpy(color_frame):
        """延迟导入 numpy，避免纯单元测试被重依赖拖慢。"""
        import numpy as np

        return np.asanyarray(color_frame.get_data())

    def _extract_detections(self, color_image, depth_frame):
        """从单帧图像中提取人体框、关键点和当前帧深度。"""
        pose_result = self.pose_model.predict(
            color_image,
            verbose=False,
            imgsz=POSE_IMGSZ,
            conf=POSE_CONF,
            iou=POSE_IOU,
            classes=[0],
            device=self.pose_device,
        )[0]

        detections = []
        if pose_result.boxes is None or pose_result.keypoints is None:
            return detections

        boxes_xyxy = pose_result.boxes.xyxy.detach().cpu().numpy()
        boxes_conf = pose_result.boxes.conf.detach().cpu().numpy()
        keypoints_xy = pose_result.keypoints.xy.detach().cpu().numpy()
        if pose_result.keypoints.conf is None:
            import numpy as np

            keypoints_conf = np.ones((len(boxes_xyxy), keypoints_xy.shape[1]), dtype="float32")
        else:
            keypoints_conf = pose_result.keypoints.conf.detach().cpu().numpy()

        for det_index in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = boxes_xyxy[det_index]
            person_box = clamp_box(x1, y1, x2, y2, FRAME_WIDTH, FRAME_HEIGHT)

            point_map = {}
            for point_name, point_index in KEYPOINT_INDEX.items():
                point_x, point_y = keypoints_xy[det_index, point_index]
                point_conf = float(keypoints_conf[det_index, point_index])
                point_map[point_name] = (int(point_x), int(point_y), point_conf)

            depth_center_x, depth_center_y = self._compute_depth_center(person_box)
            depth_m = sample_box_depth_mean(depth_frame, person_box)
            detections.append(
                {
                    "box": person_box,
                    "box_conf": float(boxes_conf[det_index]),
                    "point_map": point_map,
                    "depth_m": depth_m,
                    "depth_pixel": (depth_center_x, depth_center_y),
                }
            )

        return detections

    @staticmethod
    def _compute_depth_center(person_box):
        """直接使用人体检测框中心作为深度采样中心。"""
        return (person_box[0] + person_box[2]) // 2, (person_box[1] + person_box[3]) // 2

    def _update_tracks(self, detections):
        """把当前帧检测结果并入时间轨迹，并更新挥手状态。"""
        current_track_ids = []
        matched_track_ids = set()

        for detection in detections:
            person_box = detection["box"]
            center_x = (person_box[0] + person_box[2]) / 2.0
            center_y = (person_box[1] + person_box[3]) / 2.0
            box_width = max(1, person_box[2] - person_box[0])

            best_track_id = None
            best_match_score = -1.0

            for track_id, track in self.tracks.items():
                if track_id in matched_track_ids:
                    continue
                if self.frame_index - track["last_seen"] > TRACK_KEEP_FRAMES:
                    continue

                previous_box = track["box"]
                previous_center_x = (previous_box[0] + previous_box[2]) / 2.0
                previous_center_y = (previous_box[1] + previous_box[3]) / 2.0
                center_distance = ((center_x - previous_center_x) ** 2 + (center_y - previous_center_y) ** 2) ** 0.5
                size_gate = max(box_width, previous_box[2] - previous_box[0]) * 1.8
                iou_score = bbox_iou(person_box, previous_box)
                distance_score = max(0.0, 1.0 - center_distance / (size_gate + 1e-6))
                match_score = iou_score + 0.5 * distance_score

                if center_distance < size_gate and match_score > best_match_score:
                    best_match_score = match_score
                    best_track_id = track_id

            if best_track_id is None or best_match_score < 0.15:
                best_track_id = self._create_track(person_box)

            track = self.tracks[best_track_id]
            matched_track_ids.add(best_track_id)
            current_track_ids.append(best_track_id)

            track["box"] = person_box
            track["point_map"] = detection["point_map"]
            track["depth_pixel"] = detection["depth_pixel"]
            track["box_conf"] = detection["box_conf"]
            track["last_seen"] = self.frame_index
            track["depth_m"] = float(detection["depth_m"])

            self._update_wave_status(track, detection["point_map"])

        self._decay_unmatched_tracks(matched_track_ids)
        return current_track_ids

    def _create_track(self, person_box):
        """创建新的轨迹结构。"""
        track_id = self.next_track_id
        self.next_track_id += 1
        self.tracks[track_id] = {
            "box": person_box,
            "point_map": {},
            "depth_m": 0.0,
            "depth_pixel": (0, 0),
            "box_conf": 0.0,
            "last_seen": self.frame_index,
            "left_raised": deque(maxlen=WAVE_HISTORY),
            "right_raised": deque(maxlen=WAVE_HISTORY),
            "is_waving": False,
        }
        return track_id

    def _clone_track_candidate(self, track_id):
        """为当前帧导出一个只读候选副本，避免外部修改内部轨迹。"""
        candidate = dict(self.tracks[track_id])
        candidate["track_id"] = track_id
        return candidate

    def _attach_camera_points_to_candidates(self, depth_frame, candidates):
        """给当前候选补上相机坐标，便于上层做 odom 投影与去重。"""
        intrinsics = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()
        enriched_candidates = []

        for candidate in candidates:
            enriched_candidate = dict(candidate)
            enriched_candidate["camera_point"] = None
            depth_m = float(enriched_candidate.get("depth_m", 0.0))
            depth_pixel = enriched_candidate.get("depth_pixel")
            if depth_m > 0.0 and depth_pixel is not None:
                depth_x, depth_y = depth_pixel
                enriched_candidate["camera_point"] = tuple(
                    self.rs.rs2_deproject_pixel_to_point(
                        intrinsics,
                        [float(depth_x), float(depth_y)],
                        depth_m,
                    )
                )
            enriched_candidates.append(enriched_candidate)

        return enriched_candidates

    def _update_wave_status(self, track, point_map):
        """根据肩肘腕相对关系更新“当前人是否挥手”。"""
        left_shoulder = point_map["left_shoulder"]
        right_shoulder = point_map["right_shoulder"]
        left_elbow = point_map["left_elbow"]
        right_elbow = point_map["right_elbow"]
        left_wrist = point_map["left_wrist"]
        right_wrist = point_map["right_wrist"]

        shoulder_width = max(3.0, abs(right_shoulder[0] - left_shoulder[0]))
        margin = shoulder_width * SIMPLE_RAISE_MARGIN_SHOULDER_WIDTH

        left_visible = min(left_shoulder[2], left_elbow[2], left_wrist[2]) > KEYPOINT_VISIBILITY
        right_visible = min(right_shoulder[2], right_elbow[2], right_wrist[2]) > KEYPOINT_VISIBILITY

        left_simple_raised = left_visible and left_wrist[1] < left_shoulder[1] - margin
        right_simple_raised = right_visible and right_wrist[1] < right_shoulder[1] - margin

        track["left_raised"].append(left_simple_raised)
        track["right_raised"].append(right_simple_raised)
        track["is_waving"] = evaluate_simple_raise(track["left_raised"]) or evaluate_simple_raise(track["right_raised"])

    def _decay_unmatched_tracks(self, matched_track_ids):
        """对本帧未匹配到的轨迹做自然衰减。"""
        for track_id in list(self.tracks.keys()):
            track = self.tracks[track_id]
            if track_id not in matched_track_ids:
                track["left_raised"].append(False)
                track["right_raised"].append(False)
                track["is_waving"] = evaluate_simple_raise(track["left_raised"]) or evaluate_simple_raise(track["right_raised"])
            if self.frame_index - track["last_seen"] > TRACK_KEEP_FRAMES:
                del self.tracks[track_id]
