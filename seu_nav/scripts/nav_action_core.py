#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题导航 action wrapper 的纯逻辑工具。"""

import copy
import math

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


POSE_EQ_TOLERANCE = 1e-6
FREE_SPACE_COST = 0
UNKNOWN_SPACE_COST = -1


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


def pose_has_meaningful_header(pose) -> bool:
    """判断 pose 是否携带了上游显式填写的 header。"""
    return bool(getattr(getattr(pose, "header", None), "frame_id", ""))


def fill_pid_pose_if_empty(nav_pose, pid_pose):
    """若上游未提供 pid_pose，则直接复制 nav_pose。"""
    if pose_has_meaningful_header(pid_pose):
        return pid_pose
    return copy.deepcopy(nav_pose)


def poses_are_equivalent(left_pose, right_pose, tolerance: float = POSE_EQ_TOLERANCE) -> bool:
    """判断两个 PoseStamped 是否表达的是同一目标位姿。"""
    if not pose_has_meaningful_header(left_pose) or not pose_has_meaningful_header(right_pose):
        return False
    if left_pose.header.frame_id != right_pose.header.frame_id:
        return False

    return (
        math.isclose(left_pose.pose.position.x, right_pose.pose.position.x, rel_tol=0.0, abs_tol=tolerance)
        and math.isclose(left_pose.pose.position.y, right_pose.pose.position.y, rel_tol=0.0, abs_tol=tolerance)
        and math.isclose(left_pose.pose.position.z, right_pose.pose.position.z, rel_tol=0.0, abs_tol=tolerance)
        and math.isclose(left_pose.pose.orientation.x, right_pose.pose.orientation.x, rel_tol=0.0, abs_tol=tolerance)
        and math.isclose(left_pose.pose.orientation.y, right_pose.pose.orientation.y, rel_tol=0.0, abs_tol=tolerance)
        and math.isclose(left_pose.pose.orientation.z, right_pose.pose.orientation.z, rel_tol=0.0, abs_tol=tolerance)
        and math.isclose(left_pose.pose.orientation.w, right_pose.pose.orientation.w, rel_tol=0.0, abs_tol=tolerance)
    )


def world_to_grid(costmap_info, world_x: float, world_y: float):
    """把世界坐标换算成 costmap 栅格坐标。"""
    resolution = float(costmap_info.resolution)
    origin_x = float(costmap_info.origin.position.x)
    origin_y = float(costmap_info.origin.position.y)
    grid_x = int(math.floor((world_x - origin_x) / resolution))
    grid_y = int(math.floor((world_y - origin_y) / resolution))
    return grid_x, grid_y


def is_grid_in_bounds(costmap_info, grid_x: int, grid_y: int) -> bool:
    """判断栅格是否落在地图范围内。"""
    return 0 <= grid_x < int(costmap_info.width) and 0 <= grid_y < int(costmap_info.height)


def grid_to_world(costmap_info, grid_x: int, grid_y: int):
    """把 costmap 栅格中心换算回世界坐标。"""
    resolution = float(costmap_info.resolution)
    origin_x = float(costmap_info.origin.position.x)
    origin_y = float(costmap_info.origin.position.y)
    world_x = origin_x + (float(grid_x) + 0.5) * resolution
    world_y = origin_y + (float(grid_y) + 0.5) * resolution
    return world_x, world_y


def costmap_cell_value(costmap_message, grid_x: int, grid_y: int):
    """读取指定栅格的代价值；越界则返回 None。"""
    if not is_grid_in_bounds(costmap_message.info, grid_x, grid_y):
        return None
    width = int(costmap_message.info.width)
    index = grid_y * width + grid_x
    return int(costmap_message.data[index])


def is_free_costmap_cell(costmap_message, grid_x: int, grid_y: int) -> bool:
    """只把纯自由栅格视为可作为导航目标的候选点。"""
    cell_value = costmap_cell_value(costmap_message, grid_x, grid_y)
    if cell_value is None or cell_value == UNKNOWN_SPACE_COST:
        return False
    return cell_value == FREE_SPACE_COST


def pose_position_distance_xy(left_pose, right_pose) -> float:
    """计算两个 pose 的二维平面距离。"""
    dx = float(left_pose.pose.position.x) - float(right_pose.pose.position.x)
    dy = float(left_pose.pose.position.y) - float(right_pose.pose.position.y)
    return math.hypot(dx, dy)


def build_pose_facing_world_point(reference_pose, target_x: float, target_y: float):
    """在保留 header/z 的前提下，让 pose 朝向给定二维点。"""
    adjusted_pose = copy.deepcopy(reference_pose)
    dx = float(target_x) - float(adjusted_pose.pose.position.x)
    dy = float(target_y) - float(adjusted_pose.pose.position.y)
    if math.hypot(dx, dy) <= 1e-6:
        return adjusted_pose

    yaw = math.atan2(dy, dx)
    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    adjusted_pose.pose.orientation.x = quaternion[0]
    adjusted_pose.pose.orientation.y = quaternion[1]
    adjusted_pose.pose.orientation.z = quaternion[2]
    adjusted_pose.pose.orientation.w = quaternion[3]
    return adjusted_pose


def build_ring_offsets(layer_index: int):
    """生成第 N 层“近似圆环”上的栅格偏移。"""
    if layer_index < 0:
        raise ValueError("layer_index must be non-negative")
    if layer_index == 0:
        return [(0, 0)]

    min_radius_sq = (float(layer_index) - 0.5) ** 2
    max_radius_sq = (float(layer_index) + 0.5) ** 2
    offsets = []
    for offset_y in range(-layer_index, layer_index + 1):
        for offset_x in range(-layer_index, layer_index + 1):
            radius_sq = float(offset_x * offset_x + offset_y * offset_y)
            if min_radius_sq < radius_sq <= max_radius_sq:
                offsets.append((offset_x, offset_y))

    if offsets:
        offsets.sort(key=lambda item: math.atan2(item[1], item[0]))
        return offsets

    offsets = []
    for offset_y in range(-layer_index, layer_index + 1):
        for offset_x in range(-layer_index, layer_index + 1):
            if max(abs(offset_x), abs(offset_y)) == layer_index:
                offsets.append((offset_x, offset_y))
    offsets.sort(key=lambda item: math.atan2(item[1], item[0]))
    return offsets


def compute_plan_length(path):
    """计算一条二维路径的总长度。"""
    poses = list(getattr(path, "poses", []))
    if not poses:
        return None
    if len(poses) == 1:
        return 0.0

    total_length = 0.0
    previous_pose = poses[0]
    for current_pose in poses[1:]:
        dx = float(current_pose.pose.position.x) - float(previous_pose.pose.position.x)
        dy = float(current_pose.pose.position.y) - float(previous_pose.pose.position.y)
        total_length += math.hypot(dx, dy)
        previous_pose = current_pose
    return total_length


def resolve_accessible_pose(start_pose, requested_goal_pose, costmap_message, max_search_radius_m: float, plan_length_fn):
    """把不可达目标投影到第一层存在可达点的圆环上。"""
    if start_pose is None:
        raise ValueError("start_pose is required")
    if requested_goal_pose is None:
        raise ValueError("requested_goal_pose is required")
    if costmap_message is None:
        raise ValueError("costmap_message is required")
    if plan_length_fn is None:
        raise ValueError("plan_length_fn is required")

    info = costmap_message.info
    target_x = float(requested_goal_pose.pose.position.x)
    target_y = float(requested_goal_pose.pose.position.y)
    target_grid_x, target_grid_y = world_to_grid(info, target_x, target_y)

    requested_goal_is_free = is_free_costmap_cell(costmap_message, target_grid_x, target_grid_y)
    if requested_goal_is_free:
        requested_plan_length = plan_length_fn(requested_goal_pose)
        if requested_plan_length is not None:
            return {
                "resolved_pose": copy.deepcopy(requested_goal_pose),
                "adjusted": False,
                "ring_index": 0,
                "plan_length": float(requested_plan_length),
            }

    max_radius_cells = max(1, int(math.ceil(float(max_search_radius_m) / float(info.resolution))))
    for ring_index in range(1, max_radius_cells + 1):
        reachable_candidates = []
        for offset_x, offset_y in build_ring_offsets(ring_index):
            candidate_grid_x = target_grid_x + offset_x
            candidate_grid_y = target_grid_y + offset_y
            if not is_free_costmap_cell(costmap_message, candidate_grid_x, candidate_grid_y):
                continue

            candidate_world_x, candidate_world_y = grid_to_world(info, candidate_grid_x, candidate_grid_y)
            candidate_pose = copy.deepcopy(requested_goal_pose)
            candidate_pose.pose.position.x = candidate_world_x
            candidate_pose.pose.position.y = candidate_world_y
            candidate_pose = build_pose_facing_world_point(candidate_pose, target_x, target_y)

            candidate_plan_length = plan_length_fn(candidate_pose)
            if candidate_plan_length is None:
                continue

            start_distance = pose_position_distance_xy(start_pose, candidate_pose)
            reachable_candidates.append(
                (
                    float(candidate_plan_length),
                    float(start_distance),
                    abs(offset_x),
                    abs(offset_y),
                    candidate_pose,
                )
            )

        if reachable_candidates:
            reachable_candidates.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
            best_candidate = reachable_candidates[0]
            return {
                "resolved_pose": copy.deepcopy(best_candidate[4]),
                "adjusted": True,
                "ring_index": ring_index,
                "plan_length": best_candidate[0],
            }

    raise ValueError(
        "no reachable pose found within {:.2f}m around requested goal".format(float(max_search_radius_m))
    )
