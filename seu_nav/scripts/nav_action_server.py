#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''餐厅题导航双模式 action wrapper。

这个节点对上游暴露 `RestaurantNav` action，对下游仍然保持
`task_nav` Action Client 的调用模式不变。

与旧版本不同，POSE 目标不再在 seu_nav 内部做一次性可达点投影，
而是作为“原始请求目标 pose”直接下发给 tracer_nav，由 task_nav
基于最新地图循环刷新实际可达点并重规划。
'''

import copy
import os
import sys

import actionlib
import rospy
import yaml

from geometry_msgs.msg import PoseStamped
from tracer_nav.msg import TaskNavAction, TaskNavGoal
from seu_restaurant_msgs.msg import RestaurantNavAction, RestaurantNavResult

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from nav_action_core import resolve_waypoint_by_name


ACTION_NAME = 'restaurant_nav'
DOWNSTREAM_ACTION_NAME = 'task_nav'
DEFAULT_TIMEOUT_SEC = 120.0
ACTION_CONNECT_TIMEOUT_SEC = 10.0
ODOM_FRAME = 'odom'
WAYPOINTS_YAML_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config',
    'waypoints.yaml',
)


def build_pose_from_waypoint_dict(pose_dict):
    '''把 YAML 中的 pose 字典转换成 PoseStamped。'''
    pose = PoseStamped()
    pose.header.frame_id = ODOM_FRAME
    pose.pose.position.x = pose_dict['position']['x']
    pose.pose.position.y = pose_dict['position']['y']
    pose.pose.position.z = pose_dict['position']['z']
    pose.pose.orientation.x = pose_dict['orientation']['x']
    pose.pose.orientation.y = pose_dict['orientation']['y']
    pose.pose.orientation.z = pose_dict['orientation']['z']
    pose.pose.orientation.w = pose_dict['orientation']['w']
    return pose


class RestaurantNavActionServer:
    '''餐厅题导航 action server。'''

    def __init__(self):
        self.timeout_sec = float(rospy.get_param('~timeout', DEFAULT_TIMEOUT_SEC))
        self.waypoints_data = self._load_waypoints(WAYPOINTS_YAML_PATH)

        self.nav_client = actionlib.SimpleActionClient(DOWNSTREAM_ACTION_NAME, TaskNavAction)
        rospy.loginfo('[seu_nav] waiting for downstream task_nav action server ...')
        connected = self.nav_client.wait_for_server(rospy.Duration(ACTION_CONNECT_TIMEOUT_SEC))
        if connected:
            rospy.loginfo('[seu_nav] connected to downstream task_nav action server')
        else:
            rospy.logwarn('[seu_nav] task_nav is not ready yet, server will keep running')

        self.server = actionlib.SimpleActionServer(
            ACTION_NAME,
            RestaurantNavAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo('[seu_nav] restaurant nav action is ready: %s', ACTION_NAME)

    @staticmethod
    def _load_waypoints(yaml_path):
        '''读取航点 YAML。'''
        data = {'waypoints': {}}
        if not os.path.exists(yaml_path):
            rospy.logwarn('[seu_nav] waypoint yaml not found: %s', yaml_path)
            return data
        with open(yaml_path, 'r', encoding='utf-8') as file_obj:
            loaded = yaml.safe_load(file_obj)
        if loaded and 'waypoints' in loaded:
            data = loaded
        return data

    @staticmethod
    def _ensure_odom_pose(pose, pose_name, allow_empty_frame=False):
        '''确保传给下游 task_nav 的目标使用 odom 坐标系。'''
        frame_id = pose.header.frame_id
        if not frame_id:
            if allow_empty_frame:
                return pose
            pose.header.frame_id = ODOM_FRAME
            return pose
        if frame_id != ODOM_FRAME:
            raise ValueError('{} frame must be "{}", got "{}"'.format(pose_name, ODOM_FRAME, frame_id))
        return pose

    def _build_task_goal_from_goal(self, goal):
        '''把上游 RestaurantNav goal 转成下游 TaskNavGoal。'''
        if goal.target_mode == goal.TARGET_MODE_NAME:
            matched_name, waypoint = resolve_waypoint_by_name(self.waypoints_data, goal.target_name)
            nav_pose = build_pose_from_waypoint_dict(waypoint['nav_pose'])
            pid_pose = build_pose_from_waypoint_dict(waypoint['pid_pose'])
            rospy.loginfo('[seu_nav] resolved target_name="%s" -> "%s"', goal.target_name, matched_name)
        elif goal.target_mode == goal.TARGET_MODE_POSE:
            nav_pose = copy.deepcopy(goal.nav_pose)
            pid_pose = copy.deepcopy(goal.pid_pose)
            nav_pose = self._ensure_odom_pose(nav_pose, 'nav_pose')
            pid_pose = self._ensure_odom_pose(pid_pose, 'pid_pose', allow_empty_frame=True)
            rospy.loginfo(
                '[seu_nav] forwarding odom pose goal downstream: (%.3f, %.3f)',
                nav_pose.pose.position.x,
                nav_pose.pose.position.y,
            )
        else:
            raise ValueError('unsupported target_mode: {}'.format(goal.target_mode))

        task_goal = TaskNavGoal()
        task_goal.nav_pose = nav_pose
        task_goal.pid_pose = pid_pose
        return task_goal

    def execute_cb(self, goal):
        '''上游 action 回调。'''
        result = RestaurantNavResult(success=False, error_msg='')

        try:
            task_goal = self._build_task_goal_from_goal(goal)
        except KeyError:
            result.error_msg = 'unknown target_name: "{}"'.format(goal.target_name)
            self.server.set_aborted(result, result.error_msg)
            return
        except Exception as exc:
            result.error_msg = 'failed to build nav goal: {}'.format(exc)
            self.server.set_aborted(result, result.error_msg)
            return

        if not self.nav_client.wait_for_server(rospy.Duration(ACTION_CONNECT_TIMEOUT_SEC)):
            result.error_msg = 'downstream task_nav action is unavailable'
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
                result.error_msg = 'navigation timeout ({:.1f}s)'.format(timeout_sec)
                self.server.set_aborted(result, result.error_msg)
                return
            rate.sleep()

        downstream_result = self.nav_client.get_result()
        if downstream_result and downstream_result.success:
            result.success = True
            self.server.set_succeeded(result)
            return

        result.error_msg = 'downstream task_nav failed'
        self.server.set_aborted(result, result.error_msg)


def main():
    '''节点入口。'''
    rospy.init_node('restaurant_nav_action_server')
    RestaurantNavActionServer()
    rospy.spin()


if __name__ == '__main__':
    main()
