#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试餐厅题导航 wrapper 的纯逻辑。"""

import os
import sys
import unittest

from geometry_msgs.msg import PoseStamped


WORKSPACE_SRC = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src"
SCRIPT_DIR = os.path.join(WORKSPACE_SRC, "seu_nav", "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from nav_action_core import fill_pid_pose_if_empty, resolve_waypoint_by_name


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
        nav_pose = PoseStamped()
        nav_pose.header.frame_id = "map"
        nav_pose.pose.position.x = 1.2
        nav_pose.pose.position.y = -0.4
        nav_pose.pose.orientation.w = 1.0

        pid_pose = PoseStamped()

        filled_pid_pose = fill_pid_pose_if_empty(nav_pose, pid_pose)

        self.assertEqual(filled_pid_pose.header.frame_id, "map")
        self.assertAlmostEqual(filled_pid_pose.pose.position.x, 1.2)
        self.assertAlmostEqual(filled_pid_pose.pose.position.y, -0.4)
        self.assertAlmostEqual(filled_pid_pose.pose.orientation.w, 1.0)


if __name__ == "__main__":
    unittest.main()
