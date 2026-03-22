#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试餐厅题导航 wrapper 的纯逻辑。"""

import math
import sys
import unittest
from pathlib import Path

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from nav_action_core import (
    build_ring_offsets,
    fill_pid_pose_if_empty,
    poses_are_equivalent,
    resolve_accessible_pose,
    resolve_waypoint_by_name,
)


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z),
        1.0 - 2.0 * (quaternion.z * quaternion.z),
    )


def _make_pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _make_costmap(width, height, resolution=1.0, default_value=0):
    costmap = OccupancyGrid()
    costmap.header.frame_id = "map"
    costmap.info = MapMetaData()
    costmap.info.width = int(width)
    costmap.info.height = int(height)
    costmap.info.resolution = float(resolution)
    costmap.info.origin = Pose()
    costmap.info.origin.position = Point(0.0, 0.0, 0.0)
    costmap.info.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    costmap.data = [int(default_value)] * (width * height)
    return costmap


def _set_cost(costmap, grid_x, grid_y, cost):
    width = int(costmap.info.width)
    costmap.data[grid_y * width + grid_x] = int(cost)


class NavActionCoreTest(unittest.TestCase):
    """导航包装层核心逻辑测试。"""

    def test_resolve_waypoint_by_name_supports_case_and_underscore_fallback(self):
        """宽松匹配应兼容大小写和下划线/空格差异。"""
        waypoints = {
            "waypoints": {
                "StartPoint": {"nav_pose": {"position": {}, "orientation": {}}, "pid_pose": {"position": {}, "orientation": {}}},
                "Kitchen_0": {"nav_pose": {"position": {}, "orientation": {}}, "pid_pose": {"position": {}, "orientation": {}}},
            }
        }

        matched_name, waypoint = resolve_waypoint_by_name(waypoints, "kitchen 0")

        self.assertEqual(matched_name, "Kitchen_0")
        self.assertIn("nav_pose", waypoint)

    def test_fill_pid_pose_if_empty_copies_nav_pose(self):
        """POSE 模式未给 pid_pose 时，应直接复制 nav_pose。"""
        nav_pose = _make_pose(1.2, -0.4)
        pid_pose = PoseStamped()

        filled_pid_pose = fill_pid_pose_if_empty(nav_pose, pid_pose)

        self.assertEqual(filled_pid_pose.header.frame_id, "map")
        self.assertAlmostEqual(filled_pid_pose.pose.position.x, 1.2)
        self.assertAlmostEqual(filled_pid_pose.pose.position.y, -0.4)
        self.assertAlmostEqual(filled_pid_pose.pose.orientation.w, 1.0)

    def test_poses_are_equivalent_checks_same_pose(self):
        """显式双位姿在完全一致时，应能识别为同一目标点。"""
        left_pose = _make_pose(0.3, 0.6, yaw=0.4)
        right_pose = _make_pose(0.3, 0.6, yaw=0.4)

        self.assertTrue(poses_are_equivalent(left_pose, right_pose))

    def test_build_ring_offsets_contains_expected_first_ring_neighbors(self):
        """第一层圆环应覆盖目标点周围八邻域。"""
        offsets = set(build_ring_offsets(1))

        self.assertTrue({(-1, 0), (1, 0), (0, -1), (0, 1)}.issubset(offsets))
        self.assertTrue({(-1, -1), (1, -1), (-1, 1), (1, 1)}.issubset(offsets))

    def test_resolve_accessible_pose_keeps_original_when_directly_reachable(self):
        """原始目标点可达时，不应改写目标位姿。"""
        start_pose = _make_pose(0.5, 0.5)
        goal_pose = _make_pose(2.2, 2.7, yaw=0.3)
        costmap = _make_costmap(6, 6, resolution=1.0, default_value=0)

        def plan_length_fn(candidate_pose):
            if math.isclose(candidate_pose.pose.position.x, 2.2, abs_tol=1e-6) and math.isclose(
                candidate_pose.pose.position.y, 2.7, abs_tol=1e-6
            ):
                return 3.4
            return None

        result = resolve_accessible_pose(start_pose, goal_pose, costmap, 2.0, plan_length_fn)

        self.assertFalse(result["adjusted"])
        self.assertEqual(result["ring_index"], 0)
        self.assertAlmostEqual(result["resolved_pose"].pose.position.x, 2.2)
        self.assertAlmostEqual(result["resolved_pose"].pose.position.y, 2.7)
        self.assertAlmostEqual(_yaw_from_quaternion(result["resolved_pose"].pose.orientation), 0.3, places=6)

    def test_resolve_accessible_pose_selects_shortest_plan_on_first_reachable_ring(self):
        """一旦某层存在可达点，应只在该层内按最短全局路径择优。"""
        start_pose = _make_pose(0.5, 0.5)
        goal_pose = _make_pose(2.5, 2.5, yaw=0.0)
        costmap = _make_costmap(6, 6, resolution=1.0, default_value=100)
        _set_cost(costmap, 2, 2, 100)
        _set_cost(costmap, 2, 1, 0)
        _set_cost(costmap, 1, 2, 0)
        _set_cost(costmap, 4, 2, 0)

        plan_lengths = {
            (2.5, 1.5): 3.0,
            (1.5, 2.5): 5.0,
            (4.5, 2.5): 1.0,
        }

        def plan_length_fn(candidate_pose):
            key = (
                round(candidate_pose.pose.position.x, 1),
                round(candidate_pose.pose.position.y, 1),
            )
            return plan_lengths.get(key)

        result = resolve_accessible_pose(start_pose, goal_pose, costmap, 3.0, plan_length_fn)

        self.assertTrue(result["adjusted"])
        self.assertEqual(result["ring_index"], 1)
        self.assertAlmostEqual(result["resolved_pose"].pose.position.x, 2.5)
        self.assertAlmostEqual(result["resolved_pose"].pose.position.y, 1.5)
        self.assertAlmostEqual(result["plan_length"], 3.0)
        self.assertAlmostEqual(_yaw_from_quaternion(result["resolved_pose"].pose.orientation), math.pi / 2.0, places=6)

    def test_resolve_accessible_pose_breaks_same_length_tie_by_distance_to_start(self):
        """同层同路径长度时，应优先选离出发点更近的候选点。"""
        start_pose = _make_pose(0.5, 0.5)
        goal_pose = _make_pose(2.5, 2.5, yaw=0.0)
        costmap = _make_costmap(6, 6, resolution=1.0, default_value=100)
        _set_cost(costmap, 2, 2, 100)
        _set_cost(costmap, 2, 1, 0)
        _set_cost(costmap, 3, 2, 0)

        def plan_length_fn(candidate_pose):
            key = (
                round(candidate_pose.pose.position.x, 1),
                round(candidate_pose.pose.position.y, 1),
            )
            if key in {(2.5, 1.5), (3.5, 2.5)}:
                return 4.0
            return None

        result = resolve_accessible_pose(start_pose, goal_pose, costmap, 2.0, plan_length_fn)

        self.assertTrue(result["adjusted"])
        self.assertAlmostEqual(result["resolved_pose"].pose.position.x, 2.5)
        self.assertAlmostEqual(result["resolved_pose"].pose.position.y, 1.5)

    def test_resolve_accessible_pose_raises_when_no_candidate_is_reachable(self):
        """搜索半径内若没有可达点，应明确报错。"""
        start_pose = _make_pose(0.5, 0.5)
        goal_pose = _make_pose(2.5, 2.5)
        costmap = _make_costmap(6, 6, resolution=1.0, default_value=100)

        with self.assertRaisesRegex(ValueError, "no reachable pose found"):
            resolve_accessible_pose(start_pose, goal_pose, costmap, 2.0, lambda candidate_pose: None)


if __name__ == "__main__":
    unittest.main()
