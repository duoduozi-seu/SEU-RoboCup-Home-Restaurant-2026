#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题导航 action wrapper 的纯逻辑工具。"""

import copy


def normalize_waypoint_name(name: str) -> str:
    """统一名称格式，便于做宽松匹配。"""
    return (name or "").strip().lower().replace("_", " ")


def resolve_waypoint_by_name(waypoints_data: dict, requested_name: str):
    """按现有 service 版本规则解析航点名。"""
    waypoints = waypoints_data.get("waypoints", {})
    if requested_name in waypoints:
        return requested_name, waypoints[requested_name]

    normalized_request = normalize_waypoint_name(requested_name)
    for key, value in waypoints.items():
        if normalize_waypoint_name(key) == normalized_request:
            return key, value

    raise KeyError(requested_name)


def fill_pid_pose_if_empty(nav_pose, pid_pose):
    """若上游未提供 pid_pose，则直接复制 nav_pose。"""
    if getattr(pid_pose.header, "frame_id", ""):
        return pid_pose
    return copy.deepcopy(nav_pose)
