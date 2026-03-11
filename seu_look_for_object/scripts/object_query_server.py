#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""seu_look_for_object 运行时服务节点。

当前实现遵循你最新确认的简化方案：
1. FIND:
   - 只遍历 YAML 里给出的精细锚点，不再在锚点上旋转搜索；
   - 到达锚点后连续检测 8 帧，命中 4 帧就确认“该锚点看到了目标”；
   - 立刻计算目标 map 坐标并发布；
   - 原地对正后，直接用视频流做匀速接近到目标前 1m；
   - 距离进入 1m ± 0.15m 后直接成功退出。
2. COUNT:
   - 只遍历 YAML 里给出的精细锚点，不做抵近；
   - 每个锚点检测 15 帧；
   - 对 15 帧里的“目标个数”做投票，票数最多的作为该锚点的真实数量；
   - 所有锚点的结果直接累加。
3. RELATE / ANALYZE:
   - 暂时继续占位，后续再补。
"""

from __future__ import annotations

from pathlib import Path
import math
import re
import time
from collections import Counter
import yaml

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

try:
    import rospy
except ImportError:  # pragma: no cover
    rospy = None

try:
    import actionlib
except ImportError:  # pragma: no cover
    actionlib = None

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None

try:
    import tf2_geometry_msgs  # noqa: F401
except ImportError:  # pragma: no cover
    tf2_geometry_msgs = None

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover
    rs = None

try:
    from ultralytics import YOLO
    from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor
except ImportError:  # pragma: no cover
    YOLO = None
    YOLOEVPSegPredictor = None

try:
    from geometry_msgs.msg import PointStamped, PoseStamped, Twist
except ImportError:  # pragma: no cover
    class _Header:
        def __init__(self):
            self.frame_id = ""
            self.stamp = 0.0

    class _Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Quaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = _Quaternion()

    class PoseStamped:  # type: ignore
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    class PointStamped:  # type: ignore
        def __init__(self):
            self.header = _Header()
            self.point = _Point()

    class Twist:  # type: ignore
        def __init__(self):
            self.linear = _Point()
            self.angular = _Point()

try:
    from visualization_msgs.msg import Marker
except ImportError:  # pragma: no cover
    class Marker:  # type: ignore
        SPHERE = 2
        ADD = 0

        def __init__(self):
            self.header = type("Header", (), {"frame_id": "", "stamp": 0.0})()
            self.ns = ""
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = type(
                "Pose",
                (),
                {
                    "position": type("Point", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
                    "orientation": type("Quaternion", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})(),
                },
            )()
            self.scale = type("Scale", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
            self.color = type("Color", (), {"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0})()

try:
    from tracer_nav.msg import TaskNavAction, TaskNavGoal
except ImportError:  # pragma: no cover
    TaskNavAction = None
    TaskNavGoal = None

from seu_look_for_object.msg import ObjectDetectionResult, ObjectQueryResult
from seu_look_for_object.srv import (
    AnalyzeObject,
    AnalyzeObjectResponse,
    CountObject,
    CountObjectResponse,
    FindObject,
    FindObjectResponse,
    RelateObject,
    RelateObjectResponse,
)


ANCHOR_PATTERN = re.compile(r"^(?P<scope>.+)_(?P<index>\d+)$")
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WAYPOINT_PATH = PACKAGE_ROOT.parent / "seu_nav" / "config" / "waypoints.yaml"
DEFAULT_MODEL_PATH = PACKAGE_ROOT / "scripts" / "yoloe26" / "yoloe-26x-seg.pt"
DEFAULT_REFER_IMAGE_PATH = PACKAGE_ROOT / "scripts" / "yoloe26" / "image_prompt.JPG"
DEFAULT_REFER_BBOX = [1054.0, 1695.0, 1460.0, 2734.0]
PREVIEW_WINDOW_NAME = "LookForObject Tracking"


def normalize_token(name: str) -> str:
    """统一名称格式，便于 scope 与 YAML 名字做宽松匹配。"""

    return str(name or "").strip().lower().replace(" ", "_")


def load_waypoints_dict(yaml_path: str | Path) -> dict:
    """读取 YAML 并返回最内层的航点字典。"""

    raw_data = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8")) or {}
    if "waypoints" in raw_data and isinstance(raw_data["waypoints"], dict):
        return raw_data["waypoints"]
    return raw_data if isinstance(raw_data, dict) else {}


def match_scope_name(waypoint_dict: dict, requested_scope: str) -> str:
    """把用户给的地点名映射到 YAML 中实际的 scope 名。"""

    normalized_scope = normalize_token(requested_scope)
    for name in waypoint_dict.keys():
        if normalize_token(name) == normalized_scope:
            return name
    for name in waypoint_dict.keys():
        match = ANCHOR_PATTERN.match(name)
        if match and normalize_token(match.group("scope")) == normalized_scope:
            return match.group("scope")
    return requested_scope


def extract_anchor_names_for_scope(waypoint_dict: dict, requested_scope: str) -> list[str]:
    """提取一个 scope 下的全部精细锚点名称，并按编号排序。"""

    canonical_scope = match_scope_name(waypoint_dict, requested_scope)
    normalized_scope = normalize_token(canonical_scope)
    anchors = []
    for name in waypoint_dict.keys():
        match = ANCHOR_PATTERN.match(name)
        if not match:
            continue
        if normalize_token(match.group("scope")) != normalized_scope:
            continue
        anchors.append((int(match.group("index")), name))
    anchors.sort(key=lambda item: item[0])
    return [item[1] for item in anchors]


def vote_count_value(frame_counts: list[int]) -> int:
    """对多帧计数结果做投票。

    规则：
    1. 谁出现次数最多就选谁；
    2. 如果票数相同，取计数更大的那个，避免把明显多物体场景压成更小值。
    """

    if not frame_counts:
        return 0
    counter = Counter(frame_counts)
    return max(counter.items(), key=lambda item: (item[1], item[0]))[0]


def build_detection_dict() -> dict:
    """创建一个统一的检测字典。"""

    return {
        "label": "",
        "score": 0.0,
        "pose_valid": False,
        "pose_frame": "map",
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "bbox": [],
        "distance": 0.0,
        "center_u": 0.0,
        "center_v": 0.0,
        "image_width": 0.0,
        "image_height": 0.0,
        "debug_image": None,
    }


def build_result_dict(
    success: bool,
    status_code: str,
    message: str,
    query_mode: str,
    target_class: str,
    reference_class: str = "",
    resolved_label: str = "",
    count: int = 0,
    selected_object: dict | None = None,
    candidate_objects: list[dict] | None = None,
) -> dict:
    """统一构造 Python 侧结果字典。"""

    return {
        "success": success,
        "status_code": status_code,
        "message": message,
        "query_mode": query_mode,
        "target_class": target_class,
        "reference_class": reference_class,
        "resolved_label": resolved_label,
        "count": count,
        "selected_object": selected_object or build_detection_dict(),
        "candidate_objects": candidate_objects or [],
    }


def build_not_implemented_result(query_mode: str, target_class: str, message: str) -> dict:
    """给尚未实现的模式返回统一失败结果。"""

    return build_result_dict(
        success=False,
        status_code="NOT_IMPLEMENTED",
        message=message,
        query_mode=query_mode,
        target_class=target_class,
    )


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四元数提取偏航角。"""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, minimum: float, maximum: float) -> float:
    """限制速度或误差范围。"""

    return max(minimum, min(maximum, value))


class ObjectQueryRosNode:
    """找物 service 节点。"""

    def __init__(self):
        self.waypoint_yaml_path = Path(rospy.get_param("~anchor_yaml_path", str(DEFAULT_WAYPOINT_PATH)))
        self.waypoint_dict = load_waypoints_dict(self.waypoint_yaml_path)

        self.task_nav_action_name = rospy.get_param("~task_nav_action_name", "task_nav")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.candidate_pose_topic = rospy.get_param("~candidate_pose_topic", "~candidate_pose")
        self.candidate_marker_topic = rospy.get_param("~candidate_marker_topic", "~candidate_marker")

        self.find_frame_count = int(rospy.get_param("~find_frame_count", 8))
        self.find_hit_count = int(rospy.get_param("~find_hit_count", 4))
        self.count_frame_count = int(rospy.get_param("~count_frame_count", 15))
        self.anchor_nav_timeout = float(rospy.get_param("~anchor_nav_timeout", 120.0))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 1.0))
        self.realsense_width = int(rospy.get_param("~realsense_width", 1280))
        self.realsense_height = int(rospy.get_param("~realsense_height", 720))
        self.realsense_fps = int(rospy.get_param("~realsense_fps", 30))
        self.detection_conf = float(rospy.get_param("~detection_conf", 0.15))
        self.refer_image_path = Path(rospy.get_param("~refer_image_path", str(DEFAULT_REFER_IMAGE_PATH)))
        self.yolo_model_path = Path(rospy.get_param("~yolo_model_path", str(DEFAULT_MODEL_PATH)))
        self.refer_bbox = rospy.get_param("~refer_bbox", list(DEFAULT_REFER_BBOX))

        self.align_center_tolerance = float(rospy.get_param("~align_center_tolerance", 0.08))
        self.align_angular_kp = float(rospy.get_param("~align_angular_kp", 1.2))
        self.approach_target_distance = float(rospy.get_param("~approach_target_distance", 1.0))
        self.approach_stop_tolerance = float(rospy.get_param("~approach_stop_tolerance", 0.15))
        self.approach_linear_speed = float(rospy.get_param("~approach_linear_speed", 0.15))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.6))
        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 12.0))
        self.preview_enabled = bool(rospy.get_param("~preview_enabled", True))

        self.nav_client = None
        self.tf_buffer = None
        self.tf_listener = None
        self.cmd_vel_pub = None
        self.candidate_pose_pub = None
        self.candidate_marker_pub = None
        self.rs_pipeline = None
        self.rs_align = None
        self.color_intrinsics = None
        self.yolo_model = None
        self.visual_prompts = None

        self._init_ros_resources()

        self.find_service = rospy.Service("find_object_service", FindObject, self.handle_find)
        self.relate_service = rospy.Service("relate_object_service", RelateObject, self.handle_relate)
        self.count_service = rospy.Service("count_object_service", CountObject, self.handle_count)
        self.analyze_service = rospy.Service("analyze_object_service", AnalyzeObject, self.handle_analyze)

    def _init_ros_resources(self):
        """集中初始化 ROS 资源。"""

        if actionlib is not None and TaskNavAction is not None:
            self.nav_client = actionlib.SimpleActionClient(self.task_nav_action_name, TaskNavAction)

        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.candidate_pose_pub = rospy.Publisher(self.candidate_pose_topic, PoseStamped, queue_size=10)
        self.candidate_marker_pub = rospy.Publisher(self.candidate_marker_topic, Marker, queue_size=10)

        rospy.on_shutdown(self._shutdown_runtime)

    def _shutdown_runtime(self):
        """节点退出时尽量关干净。"""

        self._publish_zero_velocity()
        if self.rs_pipeline is not None:
            try:
                self.rs_pipeline.stop()
            except Exception:
                pass
        if self.preview_enabled and cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    def _ensure_runtime_ready(self, require_tf: bool) -> str:
        """检查视觉与导航运行时是否齐全。"""

        if getattr(self, "nav_client", None) is None:
            return "缺少 actionlib 或 tracer_nav，无法导航到精细锚点。"
        if require_tf and getattr(self, "tf_buffer", None) is None:
            return "缺少 tf2_ros，无法把目标投影到 map。"
        if rs is None:
            return "缺少 pyrealsense2，无法读取 D435i 数据。"
        if YOLO is None or YOLOEVPSegPredictor is None:
            return "缺少 ultralytics YOLOE 运行依赖。"
        if cv2 is None or np is None:
            return "缺少 cv2 或 numpy。"
        if not self.yolo_model_path.exists():
            return f"YOLO 模型文件不存在：{self.yolo_model_path}"
        if not self.refer_image_path.exists():
            return f"refer_image 文件不存在：{self.refer_image_path}"

        if self.yolo_model is None:
            rospy.loginfo("加载 YOLOE 模型：%s", self.yolo_model_path)
            self.yolo_model = YOLO(str(self.yolo_model_path))
            self.visual_prompts = self._load_visual_prompts(self.refer_image_path, self.refer_bbox)

        if self.rs_pipeline is None:
            rospy.loginfo("启动 RealSense D435i：%dx%d@%d", self.realsense_width, self.realsense_height, self.realsense_fps)
            self.rs_pipeline = rs.pipeline()
            rs_config = rs.config()
            rs_config.enable_stream(rs.stream.color, self.realsense_width, self.realsense_height, rs.format.bgr8, self.realsense_fps)
            rs_config.enable_stream(rs.stream.depth, self.realsense_width, self.realsense_height, rs.format.z16, self.realsense_fps)
            profile = self.rs_pipeline.start(rs_config)
            self.rs_align = rs.align(rs.stream.color)
            color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            self.color_intrinsics = color_stream.get_intrinsics()

        if not self.nav_client.wait_for_server(rospy.Duration(5.0)):
            return f"导航 Action Server '{self.task_nav_action_name}' 不可用。"

        return ""

    def _ensure_find_runtime_ready(self) -> str:
        """FIND 需要 tf，因为要把目标投影到 map。"""

        return self._ensure_runtime_ready(require_tf=True)

    def _ensure_count_runtime_ready(self) -> str:
        """COUNT 只需要遍历锚点和视觉，不需要 tf 投影。"""

        return self._ensure_runtime_ready(require_tf=False)

    def _load_visual_prompts(self, prompt_image_path: Path, prompt_bbox: list[float]) -> dict:
        """复用 yolo_detect_stream.py 的 refer_image 思路。"""

        prompt_image = cv2.imread(str(prompt_image_path))
        if prompt_image is None:
            raise RuntimeError(f"无法读取 refer_image：{prompt_image_path}")

        height, width = prompt_image.shape[:2]
        bbox = prompt_bbox or [0.0, 0.0, float(width), float(height)]
        return {
            "bboxes": np.array([bbox], dtype=np.float32),
            "cls": np.array([0], dtype=np.int32),
        }

    def handle_find(self, request: FindObject.Request) -> FindObjectResponse:
        """FIND 主流程入口。"""

        result = self._run_find_flow(request.target_class, request.search_scope, request.timeout_sec)
        return FindObjectResponse(result=self._to_ros_result(result))

    def handle_relate(self, request: RelateObject.Request) -> RelateObjectResponse:
        """RELATE 以后从这里继续写。

        预留逻辑：
        1. 在固定锚点视角下同时检测 target 和 reference；
        2. 做几何关系判断；
        3. 如果需要，再复用 FIND 的对正与匀速接近链路。
        """

        result = build_not_implemented_result(
            query_mode="RELATE",
            target_class=request.target_class,
            message="RELATE 模式暂未实现。后续在这里补“关系判断 + 可选接近”流程。",
        )
        result["reference_class"] = request.reference_class
        return RelateObjectResponse(result=self._to_ros_result(result))

    def handle_count(self, request: CountObject.Request) -> CountObjectResponse:
        """COUNT 主流程入口。"""

        result = self._run_count_flow(request.target_class, request.search_scope, request.timeout_sec)
        return CountObjectResponse(result=self._to_ros_result(result))

    def handle_analyze(self, request: AnalyzeObject.Request) -> AnalyzeObjectResponse:
        """ANALYZE 以后从这里继续写。

        预留逻辑：
        1. 逐锚点收集候选目标；
        2. 基于大小、距离等指标排序；
        3. 直接返回结果，不做 FIND 的 PID 接近。
        """

        result = build_not_implemented_result(
            query_mode="ANALYZE",
            target_class=request.target_class,
            message="ANALYZE 模式暂未实现。后续在这里补“候选收集 + 指标排序”流程。",
        )
        return AnalyzeObjectResponse(result=self._to_ros_result(result))

    def _run_find_flow(self, target_class: str, search_scope: str, timeout_sec: float) -> dict:
        """FIND 主流程：
        导航到锚点 -> 8 帧检测 -> 对正 -> PID 接近到 1m -> 成功。
        """

        if timeout_sec <= 0:
            return build_result_dict(False, "INVALID_QUERY", "timeout_sec 必须大于 0。", "FIND", target_class)

        runtime_error = self._ensure_find_runtime_ready()
        if runtime_error:
            return build_result_dict(False, "DEPENDENCY_MISSING", runtime_error, "FIND", target_class)

        anchor_names = extract_anchor_names_for_scope(self.waypoint_dict, search_scope)
        if not anchor_names:
            return build_result_dict(
                False,
                "UNKNOWN_SCOPE",
                f"scope '{search_scope}' 没有找到任何精细锚点（例如 {search_scope}_0）。",
                "FIND",
                target_class,
            )

        deadline = time.time() + timeout_sec
        candidate_history = []
        rospy.loginfo("开始 FIND：target=%s scope=%s anchors=%s", target_class, search_scope, anchor_names)

        for anchor_name in anchor_names:
            if time.time() >= deadline:
                break

            rospy.loginfo("前往精细锚点：%s", anchor_name)
            if not self._navigate_to_waypoint(anchor_name, min(self.anchor_nav_timeout, max(1.0, deadline - time.time()))):
                rospy.logwarn("锚点导航失败，跳过：%s", anchor_name)
                continue

            detect_result = self._detect_target_frames(target_class, self.find_frame_count, self.find_hit_count)
            if not detect_result["confirmed"]:
                rospy.loginfo("锚点 %s 未检测到目标，前往下一个锚点。", anchor_name)
                continue

            initial_detection = detect_result["last_detection"]
            candidate_history.append(initial_detection)
            self._publish_candidate(initial_detection)

            rospy.loginfo(
                "锚点 %s 检测到目标，开始原地对正。score=%.3f map=(%.3f, %.3f, %.3f)",
                anchor_name,
                initial_detection["score"],
                initial_detection["x"],
                initial_detection["y"],
                initial_detection["z"],
            )

            if not self._align_to_detection(initial_detection, target_class):
                rospy.logwarn("对正失败，继续下一个锚点。")
                continue

            final_detection = self._approach_to_target_with_constant_speed(
                target_class,
                self.approach_target_distance,
                self.approach_stop_tolerance,
            )
            if final_detection is None:
                rospy.logwarn("匀速接近失败，继续下一个锚点。")
                continue

            candidate_history.append(final_detection)
            self._publish_candidate(final_detection)
            return build_result_dict(
                success=True,
                status_code="SUCCESS",
                message="FIND 成功：已在固定锚点视角检测到目标，并匀速接近到目标前 1m。",
                query_mode="FIND",
                target_class=target_class,
                resolved_label=target_class,
                selected_object=final_detection,
                candidate_objects=candidate_history,
            )

        return build_result_dict(
            success=False,
            status_code="NOT_FOUND",
            message="所有精细锚点都遍历完毕，仍未找到目标物。",
            query_mode="FIND",
            target_class=target_class,
            candidate_objects=candidate_history,
        )

    def _run_count_flow(self, target_class: str, search_scope: str, timeout_sec: float) -> dict:
        """COUNT 主流程：
        导航到锚点 -> 连续 15 帧计数投票 -> 所有锚点累加。
        """

        if timeout_sec <= 0:
            return build_result_dict(False, "INVALID_QUERY", "timeout_sec 必须大于 0。", "COUNT", target_class)

        runtime_ready = getattr(self, "_ensure_count_runtime_ready", self._ensure_find_runtime_ready)
        runtime_error = runtime_ready()
        if runtime_error:
            return build_result_dict(False, "DEPENDENCY_MISSING", runtime_error, "COUNT", target_class)

        anchor_names = extract_anchor_names_for_scope(self.waypoint_dict, search_scope)
        if not anchor_names:
            return build_result_dict(
                False,
                "UNKNOWN_SCOPE",
                f"scope '{search_scope}' 没有找到任何精细锚点（例如 {search_scope}_0）。",
                "COUNT",
                target_class,
            )

        deadline = time.time() + timeout_sec
        total_count = 0

        for anchor_name in anchor_names:
            if time.time() >= deadline:
                break

            rospy.loginfo("COUNT 前往精细锚点：%s", anchor_name)
            if not self._navigate_to_waypoint(anchor_name, min(self.anchor_nav_timeout, max(1.0, deadline - time.time()))):
                rospy.logwarn("COUNT 锚点导航失败，跳过：%s", anchor_name)
                continue

            voted_count = self._count_target_frames(target_class, self.count_frame_count)
            rospy.loginfo("锚点 %s 的投票计数结果为：%d", anchor_name, voted_count)
            total_count += voted_count

        return build_result_dict(
            success=True,
            status_code="SUCCESS",
            message="COUNT 成功：已完成全部精细锚点的计数投票。",
            query_mode="COUNT",
            target_class=target_class,
            count=total_count,
            resolved_label=target_class,
        )

    def _get_aligned_frames(self):
        """获取一组对齐后的彩色图和深度图。"""

        frames = self.rs_pipeline.wait_for_frames()
        aligned_frames = self.rs_align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None
        color_image = np.asanyarray(color_frame.get_data())
        return {
            "color_frame": color_frame,
            "depth_frame": depth_frame,
            "color_image": color_image,
        }

    def _predict_boxes(self, color_image):
        """跑一次 YOLOE，返回当前帧的全部检测框。"""

        results = self.yolo_model.predict(
            source=color_image,
            conf=self.detection_conf,
            verbose=False,
            refer_image=str(self.refer_image_path),
            visual_prompts=self.visual_prompts,
            predictor=YOLOEVPSegPredictor,
        )
        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return []

        xyxy_list = result.boxes.xyxy.tolist()
        score_list = result.boxes.conf.tolist()
        boxes = []
        for bbox, score in zip(xyxy_list, score_list):
            boxes.append(
                {
                    "bbox": [float(value) for value in bbox],
                    "score": float(score),
                }
            )
        return boxes

    def _detect_target_frames(self, target_class: str, frame_count: int, hit_count: int) -> dict:
        """在固定锚点视角下连续检测多帧。"""

        hits = 0
        last_detection = None
        for _ in range(frame_count):
            detection = self._detect_target_once(target_class)
            if detection is not None:
                hits += 1
                last_detection = detection
            rospy.sleep(0.05)
        return {
            "confirmed": hits >= hit_count and last_detection is not None,
            "hit_count": hits,
            "last_detection": last_detection,
        }

    def _count_target_frames(self, target_class: str, frame_count: int) -> int:
        """连续多帧计数，并对结果投票。"""

        frame_counts = []
        for _ in range(frame_count):
            frame_counts.append(self._count_target_once(target_class))
            rospy.sleep(0.03)
        return vote_count_value(frame_counts)

    def _detect_target_once(self, target_class: str) -> dict | None:
        """采一帧做一次目标检测，并计算目标 map 坐标。"""

        frame_bundle = self._get_aligned_frames()
        if frame_bundle is None:
            return None

        boxes = self._predict_boxes(frame_bundle["color_image"])
        if not boxes:
            return None

        best_box = max(boxes, key=lambda item: item["score"])
        center_u = int((best_box["bbox"][0] + best_box["bbox"][2]) / 2.0)
        center_v = int((best_box["bbox"][1] + best_box["bbox"][3]) / 2.0)

        depth_m = self._read_depth_meters(frame_bundle["depth_frame"], center_u, center_v)
        if depth_m <= 0.0:
            return None

        camera_point = rs.rs2_deproject_pixel_to_point(self.color_intrinsics, [center_u, center_v], depth_m)
        map_point = self._transform_camera_point_to_map(camera_point)
        if map_point is None:
            return None

        detection = build_detection_dict()
        detection["label"] = target_class
        detection["score"] = best_box["score"]
        detection["bbox"] = best_box["bbox"]
        detection["distance"] = float(depth_m)
        detection["pose_valid"] = True
        detection["pose_frame"] = self.map_frame
        detection["x"] = float(map_point.point.x)
        detection["y"] = float(map_point.point.y)
        detection["z"] = float(map_point.point.z)
        detection["center_u"] = float(center_u)
        detection["center_v"] = float(center_v)
        detection["image_width"] = float(frame_bundle["color_image"].shape[1])
        detection["image_height"] = float(frame_bundle["color_image"].shape[0])
        detection["debug_image"] = frame_bundle["color_image"].copy()
        return detection

    def _count_target_once(self, target_class: str) -> int:
        """采一帧并返回当前帧里的目标数量。"""

        frame_bundle = self._get_aligned_frames()
        if frame_bundle is None:
            return 0

        boxes = self._predict_boxes(frame_bundle["color_image"])
        if self.preview_enabled:
            preview = frame_bundle["color_image"].copy()
            for item in boxes:
                bbox = item["bbox"]
                cv2.rectangle(preview, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
            cv2.putText(preview, f"{target_class} count={len(boxes)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow(PREVIEW_WINDOW_NAME, preview)
            cv2.waitKey(1)
        return len(boxes)

    def _align_to_detection(self, detection: dict, target_class: str) -> bool:
        """用持续视频流把目标框中心转到图像中间。"""

        miss_count = 0
        rate = rospy.Rate(self.control_rate_hz)

        while not rospy.is_shutdown():
            live_detection = self._detect_target_once(target_class)
            if live_detection is None:
                miss_count += 1
                self._publish_zero_velocity()
                if miss_count >= self.find_hit_count:
                    return False
                rate.sleep()
                continue

            miss_count = 0
            center_error = (live_detection["center_u"] - (live_detection["image_width"] / 2.0)) / max(live_detection["image_width"] / 2.0, 1.0)
            self._show_tracking_preview(live_detection, prefix="ALIGN")

            if abs(center_error) <= self.align_center_tolerance:
                self._publish_zero_velocity()
                return True

            twist = Twist()
            twist.angular.z = clamp(-self.align_angular_kp * center_error, -self.max_angular_speed, self.max_angular_speed)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self._publish_zero_velocity()
        return False

    def _approach_to_target_with_constant_speed(self, target_class: str, target_distance: float, tolerance: float) -> dict | None:
        """持续看目标并以固定线速度接近到目标前 1m。

        这里不再使用 PID。
        原因是当前流程已经先做过原地对正，靠近阶段只需要保持匀速前进并持续监控距离。
        """

        miss_count = 0
        rate = rospy.Rate(self.control_rate_hz)

        while not rospy.is_shutdown():
            live_detection = self._detect_target_once(target_class)
            if live_detection is None:
                miss_count += 1
                self._publish_zero_velocity()
                if miss_count >= self.find_hit_count:
                    return None
                rate.sleep()
                continue

            miss_count = 0
            distance_error = live_detection["distance"] - target_distance
            self._show_tracking_preview(live_detection, prefix="APPROACH")

            if abs(distance_error) <= tolerance:
                self._publish_zero_velocity()
                return live_detection

            twist = Twist()
            if distance_error > tolerance:
                twist.linear.x = self.approach_linear_speed
            else:
                twist.linear.x = -self.approach_linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self._publish_zero_velocity()
        return None

    def _show_tracking_preview(self, detection: dict, prefix: str):
        """在视频左上角叠加实时距离，并显示检测框。"""

        if not self.preview_enabled or cv2 is None or detection["debug_image"] is None:
            return

        image = detection["debug_image"].copy()
        bbox = detection["bbox"]
        cv2.rectangle(image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
        cv2.putText(
            image,
            f"{prefix} dist={detection['distance']:.2f}m target={self.approach_target_distance:.2f}m",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
        )
        cv2.imshow(PREVIEW_WINDOW_NAME, image)
        cv2.waitKey(1)

    def _read_depth_meters(self, depth_frame, center_u: int, center_v: int) -> float:
        """在检测框中心附近取一个小窗口，减少深度噪声。"""

        depth_values = []
        for du in range(-1, 2):
            for dv in range(-1, 2):
                u = max(0, center_u + du)
                v = max(0, center_v + dv)
                depth_m = float(depth_frame.get_distance(u, v))
                if depth_m > 0.05:
                    depth_values.append(depth_m)
        if not depth_values:
            return 0.0
        depth_values.sort()
        return depth_values[len(depth_values) // 2]

    def _transform_camera_point_to_map(self, camera_point: list[float]):
        """把相机坐标系下的 3D 点转换到 map。"""

        point_msg = PointStamped()
        point_msg.header.frame_id = self.camera_frame
        point_msg.header.stamp = rospy.Time(0)
        point_msg.point.x = float(camera_point[0])
        point_msg.point.y = float(camera_point[1])
        point_msg.point.z = float(camera_point[2])
        try:
            return self.tf_buffer.transform(point_msg, self.map_frame, rospy.Duration(self.tf_timeout))
        except Exception as exc:
            rospy.logwarn("相机点转 map 失败：%s", exc)
            return None

    def _navigate_to_waypoint(self, waypoint_name: str, timeout_sec: float) -> bool:
        """按 seu_nav 的方式向 task_nav 发精细锚点 goal。"""

        waypoint = self.waypoint_dict.get(waypoint_name)
        if not waypoint:
            rospy.logwarn("找不到航点：%s", waypoint_name)
            return False

        goal = TaskNavGoal()
        goal.nav_pose = self._pose_from_waypoint_record(waypoint["nav_pose"])
        goal.pid_pose = self._pose_from_waypoint_record(waypoint["pid_pose"])
        return self._send_nav_goal_and_wait(goal, timeout_sec)

    def _send_nav_goal_and_wait(self, goal, timeout_sec: float) -> bool:
        """发导航目标并等待完成。"""

        self.nav_client.send_goal(goal)
        start_time = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if time.time() - start_time > timeout_sec:
                rospy.logwarn("导航超时，取消当前 goal。")
                self.nav_client.cancel_goal()
                return False
            if self.nav_client.wait_for_result(rospy.Duration(0.05)):
                result = self.nav_client.get_result()
                return bool(getattr(result, "success", True))
            rate.sleep()
        self.nav_client.cancel_goal()
        return False

    def _pose_from_waypoint_record(self, pose_record: dict) -> PoseStamped:
        """把 YAML 里的 pose 记录转成 PoseStamped。"""

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = pose_record["position"]["x"]
        pose_msg.pose.position.y = pose_record["position"]["y"]
        pose_msg.pose.position.z = pose_record["position"]["z"]
        pose_msg.pose.orientation.x = pose_record["orientation"]["x"]
        pose_msg.pose.orientation.y = pose_record["orientation"]["y"]
        pose_msg.pose.orientation.z = pose_record["orientation"]["z"]
        pose_msg.pose.orientation.w = pose_record["orientation"]["w"]
        return pose_msg

    def _publish_zero_velocity(self):
        """停止底盘。"""

        if getattr(self, "cmd_vel_pub", None) is not None:
            self.cmd_vel_pub.publish(Twist())

    def _publish_candidate(self, detection: dict):
        """发布目标 map 坐标，方便 RViz 观察。"""

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = detection["pose_frame"]
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = detection["x"]
        pose_msg.pose.position.y = detection["y"]
        pose_msg.pose.position.z = detection["z"]
        pose_msg.pose.orientation.w = 1.0
        if getattr(self, "candidate_pose_pub", None) is not None:
            self.candidate_pose_pub.publish(pose_msg)

        marker = Marker()
        marker.header.frame_id = detection["pose_frame"]
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object_query_candidates"
        marker.id = int(time.time() * 1000) % 100000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = detection["x"]
        marker.pose.position.y = detection["y"]
        marker.pose.position.z = detection["z"]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.9
        if getattr(self, "candidate_marker_pub", None) is not None:
            self.candidate_marker_pub.publish(marker)

    def _to_ros_detection(self, detection_dict: dict) -> ObjectDetectionResult:
        """把 Python 检测字典转为 ROS msg。"""

        detection_msg = ObjectDetectionResult()
        detection_msg.label = detection_dict["label"]
        detection_msg.score = detection_dict["score"]
        detection_msg.pose_valid = detection_dict["pose_valid"]
        detection_msg.distance = detection_dict["distance"]
        detection_msg.bbox = list(detection_dict["bbox"])
        detection_msg.pose.header.frame_id = detection_dict["pose_frame"]
        detection_msg.pose.header.stamp = rospy.Time.now()
        detection_msg.pose.pose.position.x = detection_dict["x"]
        detection_msg.pose.pose.position.y = detection_dict["y"]
        detection_msg.pose.pose.position.z = detection_dict["z"]
        detection_msg.pose.pose.orientation.w = 1.0
        return detection_msg

    def _to_ros_result(self, result_dict: dict) -> ObjectQueryResult:
        """把 Python 结果字典转为统一 ROS 结果。"""

        result_msg = ObjectQueryResult()
        result_msg.success = result_dict["success"]
        result_msg.status_code = result_dict["status_code"]
        result_msg.message = result_dict["message"]
        result_msg.query_mode = result_dict["query_mode"]
        result_msg.target_class = result_dict["target_class"]
        result_msg.reference_class = result_dict["reference_class"]
        result_msg.resolved_label = result_dict["resolved_label"]
        result_msg.count = result_dict["count"]
        result_msg.selected_object = self._to_ros_detection(result_dict["selected_object"])
        result_msg.candidate_objects = [self._to_ros_detection(item) for item in result_dict["candidate_objects"]]
        return result_msg


def main():
    rospy.init_node("object_query_server")
    ObjectQueryRosNode()
    rospy.loginfo("seu_look_for_object 运行时节点已启动：FIND/COUNT 为当前有效分支。")
    rospy.spin()


if __name__ == "__main__":
    main()
