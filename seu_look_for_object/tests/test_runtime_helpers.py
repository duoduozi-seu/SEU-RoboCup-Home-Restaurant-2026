import importlib.util
import sys
import types
import unittest
from pathlib import Path


PACKAGE_ROOT = Path("/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object")
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "object_query_server.py"


def _load_server_module():
    captured_errors = []

    def _capture_logerr(*args, **kwargs):
        if not args:
            captured_errors.append("")
            return
        template = args[0]
        if len(args) == 1:
            captured_errors.append(str(template))
            return
        try:
            captured_errors.append(str(template) % args[1:])
        except Exception:
            captured_errors.append(" ".join(str(item) for item in args))

    fake_rospy = types.SimpleNamespace(
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=_capture_logerr,
        sleep=lambda *args, **kwargs: None,
        is_shutdown=lambda: False,
        Duration=lambda value=0.0: value,
        Time=types.SimpleNamespace(now=lambda: 0.0),
    )

    class FakeHeader:
        def __init__(self):
            self.frame_id = ""
            self.stamp = 0.0

    class FakePoint:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class FakeQuaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class FakePose:
        def __init__(self):
            self.position = FakePoint()
            self.orientation = FakeQuaternion()

    class FakePoseStamped:
        def __init__(self):
            self.header = FakeHeader()
            self.pose = FakePose()

    class FakePointStamped:
        def __init__(self):
            self.header = FakeHeader()
            self.point = FakePoint()

    class FakeTwist:
        def __init__(self):
            self.linear = FakePoint()
            self.angular = FakePoint()

    fake_geometry_pkg = types.ModuleType("geometry_msgs")
    fake_geometry_msg_module = types.ModuleType("geometry_msgs.msg")
    fake_geometry_msg_module.PoseStamped = FakePoseStamped
    fake_geometry_msg_module.PointStamped = FakePointStamped
    fake_geometry_msg_module.Twist = FakeTwist
    fake_geometry_pkg.msg = fake_geometry_msg_module

    class FakeMarker:
        SPHERE = 2
        ADD = 0

        def __init__(self):
            self.header = FakeHeader()
            self.ns = ""
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = FakePose()
            self.scale = FakePoint()
            self.color = types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)

    fake_visualization_msg_module = types.ModuleType("visualization_msgs.msg")
    fake_visualization_msg_module.Marker = FakeMarker

    fake_seu_msg_module = types.ModuleType("seu_look_for_object.msg")
    fake_seu_msg_module.ObjectDetectionResult = type("ObjectDetectionResult", (), {})
    fake_seu_msg_module.ObjectQueryResult = type("ObjectQueryResult", (), {})

    fake_srv_module = types.ModuleType("seu_look_for_object.srv")
    for name in [
        "AnalyzeObject",
        "AnalyzeObjectResponse",
        "CountObject",
        "CountObjectResponse",
        "FindObject",
        "FindObjectResponse",
        "RelateObject",
        "RelateObjectResponse",
    ]:
        setattr(fake_srv_module, name, type(name, (), {}))

    sys.modules["rospy"] = fake_rospy
    sys.modules["geometry_msgs"] = fake_geometry_pkg
    sys.modules["geometry_msgs.msg"] = fake_geometry_msg_module
    sys.modules["visualization_msgs.msg"] = fake_visualization_msg_module
    sys.modules["seu_look_for_object.msg"] = fake_seu_msg_module
    sys.modules["seu_look_for_object.srv"] = fake_srv_module

    spec = importlib.util.spec_from_file_location("seu_look_for_object_object_query_server", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module._captured_log_errors = captured_errors
    return module


class RuntimeHelperTest(unittest.TestCase):
    def test_extract_anchor_names_for_scope_orders_numeric_suffix(self):
        module = _load_server_module()
        waypoint_dict = {
            "Kitchen": {},
            "Kitchen_2": {},
            "Kitchen_0": {},
            "Kitchen_1": {},
            "Bedroom_0": {},
        }

        anchors = module.extract_anchor_names_for_scope(waypoint_dict, "Kitchen")

        self.assertEqual(anchors, ["Kitchen_0", "Kitchen_1", "Kitchen_2"])

    def test_vote_count_value_returns_majority_count(self):
        module = _load_server_module()

        voted_count = module.vote_count_value([0, 2, 2, 1, 2, 0, 2])

        self.assertEqual(voted_count, 2)

    def test_build_not_implemented_result_marks_failure(self):
        module = _load_server_module()

        result = module.build_not_implemented_result(
            query_mode="ANALYZE",
            target_class="box",
            message="分析模式暂未实现",
        )

        self.assertFalse(result["success"])
        self.assertEqual(result["status_code"], "NOT_IMPLEMENTED")
        self.assertEqual(result["query_mode"], "ANALYZE")
        self.assertEqual(result["target_class"], "box")
        self.assertIn("暂未实现", result["message"])

    def test_run_find_flow_traverses_anchors_until_detection_then_returns_success(self):
        module = _load_server_module()
        node = module.ObjectQueryRosNode.__new__(module.ObjectQueryRosNode)
        node.waypoint_dict = {"bedroom_0": {}, "bedroom_1": {}}
        node.anchor_nav_timeout = 30.0
        node.find_hit_count = 4
        node.find_frame_count = 8
        node.approach_target_distance = 1.0
        node.approach_stop_tolerance = 0.15
        node._ensure_find_runtime_ready = lambda: ""
        node._navigate_to_waypoint = lambda anchor_name, timeout_sec: True
        responses = iter([
            {"confirmed": False, "last_detection": None},
            {"confirmed": True, "last_detection": {"label": "apple", "score": 0.9, "pose_valid": True, "pose_frame": "map", "x": 1.0, "y": 2.0, "z": 0.0, "bbox": [1, 2, 3, 4], "distance": 1.8}},
        ])
        node._detect_target_frames = lambda target_class, frame_count, hit_count: next(responses)
        node._align_to_detection = lambda detection, target_class: True
        node._approach_to_target_with_constant_speed = lambda target_class, target_distance, tolerance: {
            "label": "apple",
            "score": 0.95,
            "pose_valid": True,
            "pose_frame": "map",
            "x": 1.2,
            "y": 2.1,
            "z": 0.0,
            "bbox": [2, 3, 4, 5],
            "distance": 1.0,
        }

        result = node._run_find_flow("apple", "bedroom", 20.0)

        self.assertTrue(result["success"])
        self.assertEqual(result["status_code"], "SUCCESS")
        self.assertEqual(result["selected_object"]["label"], "apple")

    def test_constant_speed_default_is_fifteen_centimeters_per_second(self):
        source = SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn('~approach_linear_speed", 0.15', source)
        self.assertIn("def _approach_to_target_with_constant_speed", source)

    def test_dependency_missing_logs_import_diagnostics(self):
        module = _load_server_module()
        node = module.ObjectQueryRosNode.__new__(module.ObjectQueryRosNode)
        node.nav_client = None
        node.tf_buffer = object()

        result = node._ensure_runtime_ready(require_tf=False)

        self.assertIn("缺少 actionlib", result)
        self.assertTrue(module._captured_log_errors)
        self.assertIn("actionlib_imported", module._captured_log_errors[-1])
        self.assertIn("task_nav_action_imported", module._captured_log_errors[-1])

    def test_realsense_pipeline_falls_back_to_safe_profile(self):
        module = _load_server_module()
        warnings = []

        def _capture_logwarn(*args, **kwargs):
            if not args:
                warnings.append("")
                return
            template = args[0]
            if len(args) == 1:
                warnings.append(str(template))
                return
            try:
                warnings.append(str(template) % args[1:])
            except Exception:
                warnings.append(" ".join(str(item) for item in args))

        module.rospy.logwarn = _capture_logwarn

        class FakeConfig:
            def __init__(self):
                self.streams = []

            def enable_stream(self, stream, width, height, fmt, fps):
                self.streams.append((stream, width, height, fmt, fps))

        class FakeColorProfile:
            def as_video_stream_profile(self):
                return self

            def get_intrinsics(self):
                return {"fx": 1.0, "fy": 1.0}

        class FakePipelineProfile:
            def get_stream(self, stream):
                return FakeColorProfile()

        class FakePipeline:
            def start(self, config):
                color_stream = next(item for item in config.streams if item[0] == fake_rs.stream.color)
                if color_stream[1:4] == (1280, 720, fake_rs.format.bgr8) and color_stream[4] == 30:
                    raise RuntimeError("Couldn't resolve requests")
                return FakePipelineProfile()

            def stop(self):
                return None

        class FakeStream:
            color = "color"
            depth = "depth"

        class FakeFormat:
            bgr8 = "bgr8"
            z16 = "z16"

        class FakeRs:
            stream = FakeStream()
            format = FakeFormat()

            @staticmethod
            def pipeline():
                return FakePipeline()

            @staticmethod
            def config():
                return FakeConfig()

            @staticmethod
            def align(stream):
                return ("align", stream)

        fake_rs = FakeRs()
        module.rs = fake_rs

        node = module.ObjectQueryRosNode.__new__(module.ObjectQueryRosNode)
        node.realsense_width = 1280
        node.realsense_height = 720
        node.realsense_fps = 30
        node.rs_pipeline = None
        node.rs_align = None
        node.color_intrinsics = None
        node._log_runtime_dependency_error = lambda reason, require_tf: None

        result = node._start_realsense_pipeline(require_tf=False)

        self.assertEqual(result, "")
        self.assertEqual((node.realsense_width, node.realsense_height, node.realsense_fps), (640, 480, 30))
        self.assertIsNotNone(node.rs_pipeline)
        self.assertEqual(node.color_intrinsics, {"fx": 1.0, "fy": 1.0})
        self.assertTrue(any("已回退到 640x480@30" in message for message in warnings))

    def test_run_count_flow_sums_voted_counts_from_each_anchor(self):
        module = _load_server_module()
        node = module.ObjectQueryRosNode.__new__(module.ObjectQueryRosNode)
        node.waypoint_dict = {"bedroom_0": {}, "bedroom_1": {}}
        node.anchor_nav_timeout = 30.0
        node.count_frame_count = 15
        node._ensure_count_runtime_ready = lambda: ""
        node._navigate_to_waypoint = lambda anchor_name, timeout_sec: True
        responses = iter([2, 1])
        node._count_target_frames = lambda target_class, frame_count: next(responses)

        result = node._run_count_flow("apple", "bedroom", 20.0)

        self.assertTrue(result["success"])
        self.assertEqual(result["status_code"], "SUCCESS")
        self.assertEqual(result["count"], 3)


if __name__ == "__main__":
    unittest.main()
