#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试挥手候选目标筛选与调试图像逻辑。"""

import sys
import unittest
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
DEVEL_PYTHON_DIR = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/devel/lib/python3/dist-packages"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if DEVEL_PYTHON_DIR not in sys.path:
    sys.path.insert(0, DEVEL_PYTHON_DIR)

from find_waving_customer_action_server import (
    MARKER_NS,
    MARKER_TOPIC,
    base_link_point_to_map_point,
    build_delete_marker,
    build_target_marker_array,
    build_target_marker,
    camera_point_to_base_link_point,
    deduplicate_nearby_candidates,
)
from wave_detector_core import (
    render_debug_image,
    select_nearest_waving_candidate,
    select_nearest_waving_candidates,
)
from tf.transformations import quaternion_from_euler


class WaveGeometryTest(unittest.TestCase):
    """挥手目标选择测试。"""

    def test_select_nearest_waving_candidate_prefers_smallest_positive_depth(self):
        """多人同时挥手时应选择最近且深度有效的目标。"""
        candidates = [
            {"is_waving": True, "depth_m": 2.4, "person_point_map": (2.0, 0.0, 0.0)},
            {"is_waving": True, "depth_m": 1.2, "person_point_map": (1.0, 0.0, 0.0)},
            {"is_waving": False, "depth_m": 0.8, "person_point_map": (0.5, 0.0, 0.0)},
        ]

        selected = select_nearest_waving_candidate(candidates)

        self.assertEqual(selected["depth_m"], 1.2)

    def test_select_nearest_waving_candidates_returns_two_nearest_people(self):
        """当有多个挥手人时，应返回最近的两个人且按距离排序。"""
        candidates = [
            {"is_waving": True, "depth_m": 2.4, "track_id": 1},
            {"is_waving": True, "depth_m": 1.2, "track_id": 2},
            {"is_waving": True, "depth_m": 1.8, "track_id": 3},
            {"is_waving": False, "depth_m": 0.8, "track_id": 4},
        ]

        selected = select_nearest_waving_candidates(candidates, max_count=2)

        self.assertEqual(len(selected), 2)
        self.assertEqual(selected[0]["track_id"], 2)
        self.assertEqual(selected[1]["track_id"], 3)

    def test_select_nearest_waving_candidate_ignores_invalid_depth(self):
        """深度无效的挥手目标不能被选中。"""
        candidates = [
            {"is_waving": True, "depth_m": 0.0, "person_point_map": (2.0, 0.0, 0.0)},
            {"is_waving": True, "depth_m": -1.0, "person_point_map": (1.0, 0.0, 0.0)},
        ]

        self.assertIsNone(select_nearest_waving_candidate(candidates))

    def test_render_debug_image_keeps_image_shape(self):
        """调试图像渲染后应保持原始尺寸不变。"""
        image = np.zeros((64, 96, 3), dtype=np.uint8)
        candidate = {"box": (10, 8, 40, 50), "depth_m": 1.25, "is_waving": True}

        rendered = render_debug_image(image, [candidate], candidate)

        self.assertEqual(rendered.shape, image.shape)

    def test_render_debug_image_marks_selected_candidate(self):
        """存在目标时，调试图像应被画上可见标记。"""
        image = np.zeros((64, 96, 3), dtype=np.uint8)
        candidate = {"box": (10, 8, 40, 50), "depth_m": 1.25, "is_waving": True}

        rendered = render_debug_image(image, [candidate], candidate)

        self.assertGreater(int(rendered.sum()), 0)

    def test_camera_point_to_base_link_point_applies_backward_camera_bias(self):
        """相机后移 60cm 时，base_link x 应额外叠加该后移偏置。"""
        base_link_point = camera_point_to_base_link_point((0.2, -0.1, 1.5))

        self.assertAlmostEqual(base_link_point[0], 0.9)
        self.assertAlmostEqual(base_link_point[1], -0.2)
        self.assertAlmostEqual(base_link_point[2], 0.1)

    def test_base_link_point_to_map_point_with_zero_yaw(self):
        """机器人朝向 odom x 正方向时，base_link 点应直接平移到 odom。"""
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "odom"
        robot_pose.pose.position.x = 2.0
        robot_pose.pose.position.y = 3.0
        robot_pose.pose.position.z = 0.0
        robot_pose.pose.orientation.w = 1.0

        map_point = base_link_point_to_map_point(robot_pose, (1.0, -0.5, 0.2))

        self.assertAlmostEqual(map_point.x, 3.0)
        self.assertAlmostEqual(map_point.y, 2.5)
        self.assertAlmostEqual(map_point.z, 0.2)

    def test_base_link_point_to_map_point_with_ninety_degree_yaw(self):
        """机器人左转 90 度时，base_link x 方向应落到 odom y 正方向。"""
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "odom"
        robot_pose.pose.position.x = 1.0
        robot_pose.pose.position.y = 2.0
        robot_pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0.0, 0.0, np.pi / 2.0)
        robot_pose.pose.orientation.x = quaternion[0]
        robot_pose.pose.orientation.y = quaternion[1]
        robot_pose.pose.orientation.z = quaternion[2]
        robot_pose.pose.orientation.w = quaternion[3]

        map_point = base_link_point_to_map_point(robot_pose, (1.0, 0.0, 0.0))

        self.assertAlmostEqual(map_point.x, 1.0, places=6)
        self.assertAlmostEqual(map_point.y, 3.0, places=6)

    def test_build_target_marker_uses_map_frame_and_sphere(self):
        """检测到目标人时，应构造 RViz 可见的小球 marker。"""
        point = type("PointLike", (), {"x": 1.2, "y": -0.4, "z": 0.3})()

        marker = build_target_marker(point)

        self.assertEqual(marker.header.frame_id, "odom")
        self.assertEqual(marker.ns, MARKER_NS)
        self.assertEqual(marker.id, 0)
        self.assertEqual(marker.type, Marker.SPHERE)
        self.assertEqual(marker.action, Marker.ADD)
        self.assertEqual(marker.pose.position.x, 1.2)
        self.assertEqual(marker.pose.position.y, -0.4)
        self.assertEqual(marker.pose.position.z, 0.3)
        self.assertGreater(marker.scale.x, 0.0)
        self.assertGreater(marker.color.a, 0.0)

    def test_build_target_marker_array_contains_multiple_points(self):
        """需要同时显示多人时，应生成包含多个 marker 的数组。"""
        points = [
            type("PointLike", (), {"x": 1.2, "y": -0.4, "z": 0.3})(),
            type("PointLike", (), {"x": 2.1, "y": 0.6, "z": 0.1})(),
        ]

        marker_array = build_target_marker_array(points)

        self.assertIsInstance(marker_array, MarkerArray)
        self.assertEqual(len(marker_array.markers), 2)
        self.assertEqual(marker_array.markers[0].id, 0)
        self.assertEqual(marker_array.markers[1].id, 1)
        self.assertEqual(marker_array.markers[1].pose.position.x, 2.1)

    def test_deduplicate_nearby_candidates_merges_people_within_forty_cm(self):
        """40cm 内的多个点应认作同一个人，只保留最近代表。"""
        candidates = [
            {"track_id": 1, "depth_m": 1.1, "person_point_map": type("PointLike", (), {"x": 1.0, "y": 2.0, "z": 0.0})()},
            {"track_id": 2, "depth_m": 1.3, "person_point_map": type("PointLike", (), {"x": 1.25, "y": 2.15, "z": 0.0})()},
            {"track_id": 3, "depth_m": 1.6, "person_point_map": type("PointLike", (), {"x": 2.0, "y": 2.0, "z": 0.0})()},
        ]

        unique_candidates = deduplicate_nearby_candidates(candidates, same_person_distance_m=0.4, max_count=2)

        self.assertEqual(len(unique_candidates), 2)
        self.assertEqual(unique_candidates[0]["track_id"], 1)
        self.assertEqual(unique_candidates[1]["track_id"], 3)

    def test_deduplicate_nearby_candidates_keeps_people_beyond_forty_cm(self):
        """超过 40cm 的人应保留为不同目标。"""
        candidates = [
            {"track_id": 1, "depth_m": 1.1, "person_point_map": type("PointLike", (), {"x": 1.0, "y": 2.0, "z": 0.0})()},
            {"track_id": 2, "depth_m": 1.2, "person_point_map": type("PointLike", (), {"x": 1.41, "y": 2.0, "z": 0.0})()},
        ]

        unique_candidates = deduplicate_nearby_candidates(candidates, same_person_distance_m=0.4, max_count=2)

        self.assertEqual(len(unique_candidates), 2)
        self.assertEqual(unique_candidates[0]["track_id"], 1)
        self.assertEqual(unique_candidates[1]["track_id"], 2)

    def test_build_delete_marker_clears_previous_marker(self):
        """没有目标时，应发布 DELETE marker 清理 RViz 中的旧点。"""
        marker = build_delete_marker()

        self.assertEqual(marker.header.frame_id, "odom")
        self.assertEqual(marker.ns, MARKER_NS)
        self.assertEqual(marker.id, 0)
        self.assertEqual(marker.action, Marker.DELETE)
        self.assertEqual(MARKER_TOPIC, "/restaurant_wave/target_marker")


if __name__ == "__main__":
    unittest.main()
