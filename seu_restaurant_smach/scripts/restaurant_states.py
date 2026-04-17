#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题状态机纯逻辑工具。

这个文件只放“SMACH 需要但不依赖具体 ROS action server 的小工具”，
确保最核心的几何逻辑能被单元测试直接覆盖。
"""

import math

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def odometry_to_pose_stamped(odometry_message) -> PoseStamped:
    """把导航系统发布的 Odometry 位姿提取成 PoseStamped。

    餐厅题内部统一使用 ``PoseStamped`` 传递机器人当前位置，
    这样状态机和动作节点都不用关心底层导航到底发的是哪种位姿消息。
    """
    pose = PoseStamped()
    pose.header = odometry_message.header
    pose.pose = odometry_message.pose.pose
    return pose


def reverse_pose_orientation(pose: PoseStamped) -> PoseStamped:
    """保持位置不变，并把机器人朝向反过来。"""
    current_yaw = euler_from_quaternion([
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ])[2]
    reversed_yaw = current_yaw + math.pi

    reversed_pose = PoseStamped()
    reversed_pose.header = pose.header
    reversed_pose.pose.position.x = pose.pose.position.x
    reversed_pose.pose.position.y = pose.pose.position.y
    reversed_pose.pose.position.z = pose.pose.position.z
    quaternion = quaternion_from_euler(0.0, 0.0, reversed_yaw)
    reversed_pose.pose.orientation.x = quaternion[0]
    reversed_pose.pose.orientation.y = quaternion[1]
    reversed_pose.pose.orientation.z = quaternion[2]
    reversed_pose.pose.orientation.w = quaternion[3]
    return reversed_pose


def build_bar_nav_and_pid_poses(
    start_pose: PoseStamped,
    extension_distance: float,
    pid_reverse_orientation: bool = True,
    nav_reverse_orientation: bool = True,
):
    """根据启动位姿生成“回吧台”的导航点和 PID 点。

    `pid_pose` 保留原始启动坐标，朝向可按场景选择是否反向；
    `nav_pose` 沿启动时的原始朝向前推 `extension_distance`，朝向可按场景选择是否反向。
    """
    if extension_distance < 0.0:
        raise ValueError("extension_distance must be non-negative")

    current_yaw = euler_from_quaternion([
        start_pose.pose.orientation.x,
        start_pose.pose.orientation.y,
        start_pose.pose.orientation.z,
        start_pose.pose.orientation.w,
    ])[2]
    reversed_yaw = current_yaw + math.pi
    nav_yaw = reversed_yaw if nav_reverse_orientation else current_yaw
    nav_quaternion = quaternion_from_euler(0.0, 0.0, nav_yaw)
    pid_yaw = reversed_yaw if pid_reverse_orientation else current_yaw
    pid_quaternion = quaternion_from_euler(0.0, 0.0, pid_yaw)

    pid_pose = PoseStamped()
    pid_pose.header = start_pose.header
    pid_pose.pose.position.x = start_pose.pose.position.x
    pid_pose.pose.position.y = start_pose.pose.position.y
    pid_pose.pose.position.z = start_pose.pose.position.z
    pid_pose.pose.orientation.x = pid_quaternion[0]
    pid_pose.pose.orientation.y = pid_quaternion[1]
    pid_pose.pose.orientation.z = pid_quaternion[2]
    pid_pose.pose.orientation.w = pid_quaternion[3]

    nav_pose = PoseStamped()
    nav_pose.header = start_pose.header
    nav_pose.pose.position.x = start_pose.pose.position.x + extension_distance * math.cos(current_yaw)
    nav_pose.pose.position.y = start_pose.pose.position.y + extension_distance * math.sin(current_yaw)
    nav_pose.pose.position.z = start_pose.pose.position.z
    nav_pose.pose.orientation.x = nav_quaternion[0]
    nav_pose.pose.orientation.y = nav_quaternion[1]
    nav_pose.pose.orientation.z = nav_quaternion[2]
    nav_pose.pose.orientation.w = nav_quaternion[3]
    return nav_pose, pid_pose


def build_customer_nav_pose(robot_pose: PoseStamped, person_point, stand_off_distance: float) -> PoseStamped:
    """根据顾客点和机器人当前位置，构造发给下游导航的原始请求位姿。

    当前不在状态机里提前做最终停靠点/standoff 计算；下游 tracer_nav
    会把这个原始目标点投影成当前地图上的实际可达点并持续重规划。
    这里仅保留“位置取顾客点、朝向指向顾客”的原始语义。

    `stand_off_distance` 参数为兼容旧调用链保留，当前不参与位置计算。
    """
    dx = person_point.x - robot_pose.pose.position.x
    dy = person_point.y - robot_pose.pose.position.y
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        raise ValueError("robot pose and person point are too close to build a navigation pose")

    target_yaw = math.atan2(dy, dx)

    nav_pose = PoseStamped()
    nav_pose.header.frame_id = robot_pose.header.frame_id or "odom"
    nav_pose.pose.position.x = person_point.x
    nav_pose.pose.position.y = person_point.y
    nav_pose.pose.position.z = 0.0

    quaternion = quaternion_from_euler(0.0, 0.0, target_yaw)
    nav_pose.pose.orientation.x = quaternion[0]
    nav_pose.pose.orientation.y = quaternion[1]
    nav_pose.pose.orientation.z = quaternion[2]
    nav_pose.pose.orientation.w = quaternion[3]
    return nav_pose
