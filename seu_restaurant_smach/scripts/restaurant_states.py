#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题状态机纯逻辑工具。

这个文件只放“SMACH 需要但不依赖具体 ROS action server 的小工具”，
确保最核心的几何逻辑能被单元测试直接覆盖。
"""

import math

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def odometry_to_pose_stamped(odometry_message) -> PoseStamped:
    """把导航系统发布的 Odometry 位姿提取成 PoseStamped。

    餐厅题内部统一使用 ``PoseStamped`` 传递机器人当前位置，
    这样状态机和动作节点都不用关心底层导航到底发的是哪种位姿消息。
    """
    pose = PoseStamped()
    pose.header = odometry_message.header
    pose.pose = odometry_message.pose.pose
    return pose


def build_customer_nav_pose(robot_pose: PoseStamped, person_point, stand_off_distance: float) -> PoseStamped:
    """根据顾客点和机器人当前位置，构造第一次靠近顾客的停靠位姿。

    规则固定：
    1. 停靠点落在“机器人 -> 顾客”的连线上；
    2. 与顾客保持 ``stand_off_distance`` 米；
    3. 机器人朝向顾客。
    """
    dx = person_point.x - robot_pose.pose.position.x
    dy = person_point.y - robot_pose.pose.position.y
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        raise ValueError("robot pose and person point are too close to build a navigation pose")

    unit_x = dx / distance
    unit_y = dy / distance
    target_yaw = math.atan2(dy, dx)

    nav_pose = PoseStamped()
    nav_pose.header.frame_id = robot_pose.header.frame_id or "map"
    nav_pose.pose.position.x = person_point.x - unit_x * stand_off_distance
    nav_pose.pose.position.y = person_point.y - unit_y * stand_off_distance
    nav_pose.pose.position.z = 0.0

    quaternion = quaternion_from_euler(0.0, 0.0, target_yaw)
    nav_pose.pose.orientation.x = quaternion[0]
    nav_pose.pose.orientation.y = quaternion[1]
    nav_pose.pose.orientation.z = quaternion[2]
    nav_pose.pose.orientation.w = quaternion[3]
    return nav_pose
