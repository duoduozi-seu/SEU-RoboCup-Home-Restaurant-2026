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
# POSE_MODEL_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), "models", "yolo26m-pose.pt")
POSE_MODEL_PATH = '/home/duo/GPSR_ws_zyd/res_demo/yolo26m-pose.pt'

FRAME_WIDTH = 848
FRAME_HEIGHT = 480
FRAME_FPS = 30
POSE_IMGSZ = 960
POSE_CONF = 0.35
POSE_IOU = 0.45
KEYPOINT_VISIBILITY = 0.35
TRACK_KEEP_FRAMES = 18
WAVE_HISTORY = 14
DEPTH_SAMPLE_RADIUS = 6
DEPTH_FILTER_WINDOW = 5
SIMPLE_RAISE_REQUIRED_FRAMES = 10
SIMPLE_RAISE_MARGIN_SHOULDER_WIDTH = 0.10
DEPTH_CENTER_BOX_Y_RATIO = 0.55

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


def sample_depth_median(depth_frame, center_x, center_y, radius=DEPTH_SAMPLE_RADIUS):
    """对局部小窗口取深度中值，减少单点抖动。"""
    profile = depth_frame.get_profile().as_video_stream_profile()
    width = profile.width()
    height = profile.height()
    center_x = int(center_x)
    center_y = int(center_y)
    valid_depths = []

    for pixel_y in range(max(0, center_y - radius), min(height, center_y + radius + 1)):
        for pixel_x in range(max(0, center_x - radius), min(width, center_x + radius + 1)):
            depth_value = depth_frame.get_distance(pixel_x, pixel_y)
            if depth_value > 0.0:
                valid_depths.append(depth_value)

    if not valid_depths:
        return 0.0
    valid_depths.sort()
    return float(valid_depths[len(valid_depths) // 2])


def render_debug_image(color_image, candidates, selected_candidate):
    """把当前候选人与最终选中目标简单画到调试图像上。

    约定：
    1. 所有当前候选人都会画框，方便看见检测是否稳定；
    2. 已满足挥手条件的人用黄色框；
    3. 当前最终被选中的最近挥手人用绿色框；
    4. 若没有候选人，直接返回原图副本。
    """
    import cv2

    debug_image = color_image.copy()

    for candidate in candidates:
        box = candidate.get("box")
        if box is None:
            continue

        x1, y1, x2, y2 = [int(value) for value in box]
        is_selected = candidate is selected_candidate
        is_waving = bool(candidate.get("is_waving", False))

        color = (160, 160, 160)
        thickness = 1
        label = "person"

        if is_waving:
            color = (0, 255, 255)
            thickness = 2
            label = "waving"
        if is_selected:
            color = (0, 255, 0)
            thickness = 3
            label = "selected"

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

    这个类只提供“每处理一帧，输出当前最近挥手人”的能力。
    """

    def __init__(self, model_path=POSE_MODEL_PATH):
        import pyrealsense2 as rs
        import torch
        from ultralytics import YOLO

        self.rs = rs
        self.pose_device = 0 if torch.cuda.is_available() else "cpu"
        if not os.path.exists(model_path):
            raise FileNotFoundError("wave model not found: {}".format(model_path))

        self.pose_model = YOLO(model_path)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FRAME_FPS)
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FRAME_FPS)
        self.align_to_color = rs.align(rs.stream.color)
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling_filter = rs.hole_filling_filter()
        self.pipeline.start(config)

        self.frame_index = 0
        self.next_track_id = 1
        self.tracks = {}

    def close(self):
        """关闭 RealSense 流。"""
        try:
            self.pipeline.stop()
        except Exception:
            pass

    def process_once(self):
        """处理一帧，并返回当前最近挥手人的相机坐标与深度。"""
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
        selected = select_nearest_waving_candidate(current_candidates)
        debug_image = render_debug_image(color_image, current_candidates, selected)
        if selected is None:
            return {
                "selected_candidate": None,
                "current_candidates": current_candidates,
                "debug_image": debug_image,
            }
        return {
            "selected_candidate": selected,
            "current_candidates": current_candidates,
            "debug_image": debug_image,
        }

    @staticmethod
    def _frame_to_numpy(color_frame):
        """延迟导入 numpy，避免纯单元测试被重依赖拖慢。"""
        import numpy as np

        return np.asanyarray(color_frame.get_data())

    def _extract_detections(self, color_image, depth_frame):
        """从单帧图像中提取人体框、关键点和稳定深度。"""
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
            if (person_box[2] - person_box[0]) < 50 or (person_box[3] - person_box[1]) < 120:
                continue

            point_map = {}
            for point_name, point_index in KEYPOINT_INDEX.items():
                point_x, point_y = keypoints_xy[det_index, point_index]
                point_conf = float(keypoints_conf[det_index, point_index])
                point_map[point_name] = (int(point_x), int(point_y), point_conf)

            depth_center_x, depth_center_y = self._compute_depth_center(person_box, point_map)
            depth_m = sample_depth_median(depth_frame, depth_center_x, depth_center_y)
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
    def _compute_depth_center(person_box, point_map):
        """优先在躯干中部采样深度，减少头肩边缘抖动。"""
        left_shoulder = point_map["left_shoulder"]
        right_shoulder = point_map["right_shoulder"]
        left_hip = point_map["left_hip"]
        right_hip = point_map["right_hip"]

        shoulder_points = [point for point in (left_shoulder, right_shoulder) if point[2] > KEYPOINT_VISIBILITY]
        hip_points = [point for point in (left_hip, right_hip) if point[2] > KEYPOINT_VISIBILITY]

        box_center_x = (person_box[0] + person_box[2]) // 2
        box_center_y = int(person_box[1] + (person_box[3] - person_box[1]) * DEPTH_CENTER_BOX_Y_RATIO)

        if len(shoulder_points) == 2 and len(hip_points) == 2:
            shoulder_center_x = sum(point[0] for point in shoulder_points) / 2.0
            shoulder_center_y = sum(point[1] for point in shoulder_points) / 2.0
            hip_center_x = sum(point[0] for point in hip_points) / 2.0
            hip_center_y = sum(point[1] for point in hip_points) / 2.0
            return int((shoulder_center_x + hip_center_x) / 2.0), int((shoulder_center_y + hip_center_y) / 2.0)

        if len(hip_points) == 2:
            return int(sum(point[0] for point in hip_points) / 2.0), int(sum(point[1] for point in hip_points) / 2.0)

        return box_center_x, box_center_y

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

            if detection["depth_m"] > 0.0:
                track["depth_history"].append(float(detection["depth_m"]))
            if track["depth_history"]:
                sorted_depths = sorted(track["depth_history"])
                track["depth_m"] = float(sorted_depths[len(sorted_depths) // 2])
            else:
                track["depth_m"] = 0.0

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
            "depth_history": deque(maxlen=DEPTH_FILTER_WINDOW),
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

        shoulder_width = max(20.0, abs(right_shoulder[0] - left_shoulder[0]))
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
