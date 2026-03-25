#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题导航双模式 action wrapper。

这个节点对上游暴露 `RestaurantNav` action，对下游仍然保持
`task_nav` Action Client 的调用模式不变。
"""

import copy
import os
import sys

import actionlib
import rospy
import tf2_ros
import yaml

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from tracer_nav.msg import TaskNavAction, TaskNavGoal
from seu_restaurant_msgs.msg import RestaurantNavAction, RestaurantNavResult

# 这里显式把当前脚本目录加入模块搜索路径。
# 这样无论从哪个工作目录直接启动这个 scripts 文件，都能稳定导入同目录辅助模块。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from nav_action_core import (
    compute_plan_length,
    pose_has_meaningful_header,
    poses_are_equivalent,
    resolve_accessible_pose,
    resolve_waypoint_by_name,
)


ACTION_NAME = "restaurant_nav"
DOWNSTREAM_ACTION_NAME = "task_nav"
DEFAULT_TIMEOUT_SEC = 120.0
ACTION_CONNECT_TIMEOUT_SEC = 10.0
MAP_FRAME = "map"
ROBOT_BASE_FRAME = "base_link"
GLOBAL_COSTMAP_TOPIC = "/move_base/global_costmap/costmap"
MAKE_PLAN_SERVICE_NAME = "/move_base/make_plan"
DEFAULT_GOAL_SEARCH_RADIUS_M = 3.5
WAIT_FOR_POSE_TIMEOUT_SEC = 4.0
WAIT_FOR_COSTMAP_TIMEOUT_SEC = 4.0
MAKE_PLAN_TIMEOUT_SEC = 2.0
MAKE_PLAN_TOLERANCE_M = 0.0
WAYPOINTS_YAML_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config",
    "waypoints.yaml",
)


def build_pose_from_waypoint_dict(pose_dict):
    """把 YAML 中的 pose 字典转换成 PoseStamped。"""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = pose_dict["position"]["x"]
    pose.pose.position.y = pose_dict["position"]["y"]
    pose.pose.position.z = pose_dict["position"]["z"]
    pose.pose.orientation.x = pose_dict["orientation"]["x"]
    pose.pose.orientation.y = pose_dict["orientation"]["y"]
    pose.pose.orientation.z = pose_dict["orientation"]["z"]
    pose.pose.orientation.w = pose_dict["orientation"]["w"]
    return pose


def transform_to_pose_stamped(transform_message):
    """把 TF 变换提取成 PoseStamped。"""
    pose = PoseStamped()
    pose.header.stamp = transform_message.header.stamp
    pose.header.frame_id = transform_message.header.frame_id
    pose.pose.position.x = transform_message.transform.translation.x
    pose.pose.position.y = transform_message.transform.translation.y
    pose.pose.position.z = transform_message.transform.translation.z
    pose.pose.orientation = transform_message.transform.rotation
    return pose


class RestaurantNavActionServer:
    """餐厅题导航 action server。"""

    def __init__(self):
        self.timeout_sec = float(rospy.get_param("~timeout", DEFAULT_TIMEOUT_SEC))
        self.goal_search_radius_m = float(rospy.get_param("~goal_search_radius_m", DEFAULT_GOAL_SEARCH_RADIUS_M))
        self.make_plan_tolerance_m = float(rospy.get_param("~make_plan_tolerance_m", MAKE_PLAN_TOLERANCE_M))
        self.waypoints_data = self._load_waypoints(WAYPOINTS_YAML_PATH)
        self.current_pose = None
        self.global_costmap = None
        self.make_plan_client = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(GLOBAL_COSTMAP_TOPIC, OccupancyGrid, self._costmap_callback, queue_size=1)

        self.nav_client = actionlib.SimpleActionClient(DOWNSTREAM_ACTION_NAME, TaskNavAction)
        rospy.loginfo("[seu_nav] waiting for downstream task_nav action server ...")
        connected = self.nav_client.wait_for_server(rospy.Duration(ACTION_CONNECT_TIMEOUT_SEC))
        if connected:
            rospy.loginfo("[seu_nav] connected to downstream task_nav action server")
        else:
            rospy.logwarn("[seu_nav] task_nav is not ready yet, server will keep running")

        self.server = actionlib.SimpleActionServer(
            ACTION_NAME,
            RestaurantNavAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[seu_nav] restaurant nav action is ready: %s", ACTION_NAME)

    def _costmap_callback(self, message):
        """缓存最新的全局代价地图。"""
        self.global_costmap = message

    @staticmethod
    def _load_waypoints(yaml_path):
        """读取航点 YAML。"""
        data = {"waypoints": {}}
        if not os.path.exists(yaml_path):
            rospy.logwarn("[seu_nav] waypoint yaml not found: %s", yaml_path)
            return data
        with open(yaml_path, "r", encoding="utf-8") as file_obj:
            loaded = yaml.safe_load(file_obj)
        if loaded and "waypoints" in loaded:
            data = loaded
        return data

    def _get_current_pose(self):
        """获取当前机器人位姿。"""
        transform_message = self.tf_buffer.lookup_transform(
            MAP_FRAME,
            ROBOT_BASE_FRAME,
            rospy.Time(0),
            rospy.Duration(WAIT_FOR_POSE_TIMEOUT_SEC),
        )
        self.current_pose = transform_to_pose_stamped(transform_message)
        return copy.deepcopy(self.current_pose)

    def _get_global_costmap(self):
        """获取最新全局代价地图，必要时阻塞等待一次。"""
        if self.global_costmap is None:
            self.global_costmap = rospy.wait_for_message(
                GLOBAL_COSTMAP_TOPIC,
                OccupancyGrid,
                timeout=WAIT_FOR_COSTMAP_TIMEOUT_SEC,
            )
        return self.global_costmap

    def _get_make_plan_client(self):
        """延迟创建 /move_base/make_plan client。"""
        if self.make_plan_client is None:
            rospy.wait_for_service(MAKE_PLAN_SERVICE_NAME, timeout=MAKE_PLAN_TIMEOUT_SEC)
            self.make_plan_client = rospy.ServiceProxy(MAKE_PLAN_SERVICE_NAME, GetPlan)
        return self.make_plan_client

    def _plan_length_from_current_pose(self, goal_pose):
        """调用 move_base 的 make_plan，返回当前位姿到目标位姿的路径长度。"""
        start_pose = self._get_current_pose()
        request = GetPlanRequest()
        request.start = start_pose
        request.goal = copy.deepcopy(goal_pose)
        request.start.header.stamp = rospy.Time.now()
        request.goal.header.stamp = rospy.Time.now()
        request.tolerance = self.make_plan_tolerance_m

        response = self._get_make_plan_client().call(request)
        return compute_plan_length(response.plan)

    def _should_sync_pid_to_nav(self, nav_pose, pid_pose):
        """判断当前 goal 是否应把 nav_pose 深拷贝给 pid_pose。"""
        return (not pose_has_meaningful_header(pid_pose)) or poses_are_equivalent(pid_pose, nav_pose)

    def _resolve_pose_mode_goal(self, nav_pose, pid_pose):
        """在 POSE 模式下，必要时把不可达目标改写成最近可达点。"""
        reachable_info = resolve_accessible_pose(
            start_pose=self._get_current_pose(),
            requested_goal_pose=nav_pose,
            costmap_message=self._get_global_costmap(),
            max_search_radius_m=self.goal_search_radius_m,
            plan_length_fn=self._plan_length_from_current_pose,
        )
        resolved_nav_pose = reachable_info["resolved_pose"]

        if reachable_info["adjusted"]:
            rospy.loginfo(
                "[seu_nav] adjusted target pose at ring=%d, requested=(%.3f, %.3f), resolved=(%.3f, %.3f), plan_length=%.3f",
                reachable_info["ring_index"],
                nav_pose.pose.position.x,
                nav_pose.pose.position.y,
                resolved_nav_pose.pose.position.x,
                resolved_nav_pose.pose.position.y,
                reachable_info["plan_length"],
            )
        else:
            rospy.loginfo(
                "[seu_nav] requested pose is directly reachable: (%.3f, %.3f), plan_length=%.3f",
                resolved_nav_pose.pose.position.x,
                resolved_nav_pose.pose.position.y,
                reachable_info["plan_length"],
            )

        if self._should_sync_pid_to_nav(nav_pose, pid_pose):
            resolved_pid_pose = copy.deepcopy(resolved_nav_pose)
        else:
            resolved_pid_pose = pid_pose
        return resolved_nav_pose, resolved_pid_pose

    def _build_task_goal_from_goal(self, goal):
        """把上游 RestaurantNav goal 转成下游 TaskNavGoal。"""
        if goal.target_mode == goal.TARGET_MODE_NAME:
            matched_name, waypoint = resolve_waypoint_by_name(self.waypoints_data, goal.target_name)
            nav_pose = build_pose_from_waypoint_dict(waypoint["nav_pose"])
            pid_pose = build_pose_from_waypoint_dict(waypoint["pid_pose"])
            rospy.loginfo('[seu_nav] resolved target_name="%s" -> "%s"', goal.target_name, matched_name)
        elif goal.target_mode == goal.TARGET_MODE_POSE:
            nav_pose, pid_pose = self._resolve_pose_mode_goal(goal.nav_pose, goal.pid_pose)
        else:
            raise ValueError("unsupported target_mode: {}".format(goal.target_mode))

        task_goal = TaskNavGoal()
        task_goal.nav_pose = nav_pose
        task_goal.pid_pose = pid_pose
        return task_goal

    def execute_cb(self, goal):
        """上游 action 回调。"""
        result = RestaurantNavResult(success=False, error_msg="")

        try:
            task_goal = self._build_task_goal_from_goal(goal)
        except KeyError:
            result.error_msg = 'unknown target_name: "{}"'.format(goal.target_name)
            self.server.set_aborted(result, result.error_msg)
            return
        except Exception as exc:
            result.error_msg = "failed to build nav goal: {}".format(exc)
            self.server.set_aborted(result, result.error_msg)
            return

        if not self.nav_client.wait_for_server(rospy.Duration(ACTION_CONNECT_TIMEOUT_SEC)):
            result.error_msg = "downstream task_nav action is unavailable"
            self.server.set_aborted(result, result.error_msg)
            return

        timeout_sec = goal.timeout_sec if goal.timeout_sec > 0.0 else self.timeout_sec
        self.nav_client.send_goal(task_goal)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.nav_client.cancel_goal()
                self.server.set_preempted()
                return

            if self.nav_client.wait_for_result(rospy.Duration(0.1)):
                break

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout_sec:
                self.nav_client.cancel_goal()
                result.error_msg = "navigation timeout ({:.1f}s)".format(timeout_sec)
                self.server.set_aborted(result, result.error_msg)
                return
            rate.sleep()

        downstream_result = self.nav_client.get_result()
        if downstream_result and downstream_result.success:
            result.success = True
            self.server.set_succeeded(result)
            return

        result.error_msg = "downstream task_nav failed"
        self.server.set_aborted(result, result.error_msg)


def main():
    """节点入口。"""
    rospy.init_node("restaurant_nav_action_server")
    RestaurantNavActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
