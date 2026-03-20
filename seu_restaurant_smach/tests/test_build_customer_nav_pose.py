#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试餐厅状态机中与位姿相关的纯几何逻辑。"""

import math
import sys
import unittest
from pathlib import Path

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from restaurant_states import build_customer_nav_pose, odometry_to_pose_stamped, reverse_pose_orientation


def _yaw_from_quaternion(quaternion):
    """从二维导航四元数中恢复 yaw。"""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z),
        1.0 - 2.0 * (quaternion.z * quaternion.z),
    )


class BuildCustomerNavPoseTest(unittest.TestCase):
    """顾客导航位姿构造测试。"""

    def test_build_customer_nav_pose_moves_back_from_person_and_faces_person(self):
        """机器人应停在顾客前方固定距离处，并保持朝向顾客。"""
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "map"
        robot_pose.pose.position.x = 0.0
        robot_pose.pose.position.y = 0.0
        robot_pose.pose.orientation.w = 1.0

        person_point = Point()
        person_point.x = 2.0
        person_point.y = 0.0
        person_point.z = 0.0

        nav_pose = build_customer_nav_pose(robot_pose, person_point, 1.0)

        self.assertEqual(nav_pose.header.frame_id, "map")
        self.assertTrue(math.isclose(nav_pose.pose.position.x, 1.0, rel_tol=0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(nav_pose.pose.position.y, 0.0, rel_tol=0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(_yaw_from_quaternion(nav_pose.pose.orientation), 0.0, rel_tol=0.0, abs_tol=1e-6))

    def test_build_customer_nav_pose_keeps_offset_along_robot_to_person_line(self):
        """顾客不在正前方时，停靠点仍应沿机器人到顾客连线后退。"""
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "map"
        robot_pose.pose.position.x = 1.0
        robot_pose.pose.position.y = 1.0
        robot_pose.pose.orientation.w = 1.0

        person_point = Point()
        person_point.x = 4.0
        person_point.y = 5.0
        person_point.z = 0.0

        nav_pose = build_customer_nav_pose(robot_pose, person_point, 2.0)

        dx = person_point.x - robot_pose.pose.position.x
        dy = person_point.y - robot_pose.pose.position.y
        distance = math.hypot(dx, dy)
        unit_x = dx / distance
        unit_y = dy / distance

        self.assertTrue(
            math.isclose(nav_pose.pose.position.x, person_point.x - unit_x * 2.0, rel_tol=0.0, abs_tol=1e-6)
        )
        self.assertTrue(
            math.isclose(nav_pose.pose.position.y, person_point.y - unit_y * 2.0, rel_tol=0.0, abs_tol=1e-6)
        )

    def test_odometry_to_pose_stamped_copies_header_and_pose(self):
        """里程计消息应被无损转成 PoseStamped，方便餐厅流程统一复用。"""
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = 1.5
        odom.pose.pose.position.y = -0.4
        odom.pose.pose.position.z = 0.2
        odom.pose.pose.orientation.z = 0.3
        odom.pose.pose.orientation.w = 0.95

        pose = odometry_to_pose_stamped(odom)

        self.assertEqual(pose.header.frame_id, "map")
        self.assertTrue(math.isclose(pose.pose.position.x, 1.5, rel_tol=0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(pose.pose.position.y, -0.4, rel_tol=0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(pose.pose.orientation.z, 0.3, rel_tol=0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(pose.pose.orientation.w, 0.95, rel_tol=0.0, abs_tol=1e-6))

    def test_reverse_pose_orientation_keeps_position_and_turns_back(self):
        """回吧台位姿应保留原位置，并把朝向反过来。"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.2
        pose.pose.position.y = -0.8
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        reversed_pose = reverse_pose_orientation(pose)

        self.assertEqual(reversed_pose.header.frame_id, "map")
        self.assertTrue(math.isclose(reversed_pose.pose.position.x, 1.2, rel_tol=0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(reversed_pose.pose.position.y, -0.8, rel_tol=0.0, abs_tol=1e-6))
        self.assertTrue(
            math.isclose(abs(_yaw_from_quaternion(reversed_pose.pose.orientation)), math.pi, rel_tol=0.0, abs_tol=1e-6)
        )


if __name__ == "__main__":
    unittest.main()
