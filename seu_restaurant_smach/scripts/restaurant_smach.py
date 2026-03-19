#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题主状态机入口。

这个文件故意把核心流程按比赛执行顺序从上往下排开：
1. 找挥手人；
2. 计算第一次靠近顾客的导航位姿；
3. 导航去顾客；
4. 微调并接单；
5. 回吧台复述；
6. 抓取占位；
7. 返回顾客；
8. 固定播报结束。
"""

import os
import sys

import actionlib
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from seu_restaurant_msgs.msg import (
    AlignAndTakeOrderAction,
    AlignAndTakeOrderGoal,
    FindWavingCustomerAction,
    FindWavingCustomerGoal,
    RestaurantNavAction,
    RestaurantNavGoal,
)
from seu_voice.srv import seu_voice as VoiceSrv
from seu_voice.srv import seu_voiceRequest

# 显式补上当前脚本目录，确保直接运行 scripts 文件时，
# 同目录工具模块也能稳定导入。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from restaurant_states import build_customer_nav_pose


WAVE_ACTION_NAME = "find_waving_customer"
NAV_ACTION_NAME = "restaurant_nav"
INTERACTION_ACTION_NAME = "align_and_take_order"
VOICE_SERVICE_NAME = "voice_service"
ROBOT_POSE_TOPIC = "/robot_pose"

BAR_TARGET_NAME = "StartPoint"
WAVE_TIMEOUT_SEC = 45.0
WAVE_ANGULAR_SPEED_RAD_S = 0.15
NAV_TIMEOUT_SEC = 120.0
STAND_OFF_DISTANCE_M = 1.0
ALIGN_TIMEOUT_SEC = 20.0
DIALOG_TIMEOUT_SEC = 20.0
ORDER_MAX_ROUNDS = 2
ASK_ORDER_TEXT = "Hello. What would you like to order?"
ORDER_REPEAT_PREFIX = "The customer ordered: "
FINAL_DELIVER_TEXT = "I am back. Please take your order. Enjoy your meal."


def build_nav_goal_from_pose(pose: PoseStamped) -> RestaurantNavGoal:
    """构造 POSE 模式导航 goal。"""
    goal = RestaurantNavGoal()
    goal.target_mode = RestaurantNavGoal.TARGET_MODE_POSE
    goal.nav_pose = pose
    goal.timeout_sec = NAV_TIMEOUT_SEC
    return goal


def build_nav_goal_from_name(target_name: str) -> RestaurantNavGoal:
    """构造 NAME 模式导航 goal。"""
    goal = RestaurantNavGoal()
    goal.target_mode = RestaurantNavGoal.TARGET_MODE_NAME
    goal.target_name = target_name
    goal.timeout_sec = NAV_TIMEOUT_SEC
    return goal


def say_once(text: str):
    """直接调用现有语音服务播报一句话。"""
    rospy.wait_for_service(VOICE_SERVICE_NAME, timeout=2.0)
    client = rospy.ServiceProxy(VOICE_SERVICE_NAME, VoiceSrv)
    response = client.call(seu_voiceRequest(text=text))
    if not response.state:
        raise RuntimeError("voice service failed: {}".format(response.errormsg))


class FindWavingCustomerState(smach.State):
    """调用挥手搜索 action，返回最近顾客点。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"], output_keys=["customer_point_map"])

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(WAVE_ACTION_NAME, FindWavingCustomerAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        for _ in range(2):
            goal = FindWavingCustomerGoal(
                timeout_sec=WAVE_TIMEOUT_SEC,
                angular_speed_rad_s=WAVE_ANGULAR_SPEED_RAD_S,
            )
            client.send_goal(goal)
            client.wait_for_result()
            result = client.get_result()
            if result is not None and result.success:
                userdata.customer_point_map = result.person_point_map
                return "success"
        return "failure"


class BuildCustomerNavPoseState(smach.State):
    """根据顾客点和当前机器人位姿，算第一次靠近顾客的位姿。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["customer_point_map"],
            output_keys=["customer_nav_pose"],
        )

    def execute(self, userdata):
        try:
            robot_pose = rospy.wait_for_message(ROBOT_POSE_TOPIC, PoseStamped, timeout=2.0)
            userdata.customer_nav_pose = build_customer_nav_pose(
                robot_pose,
                userdata.customer_point_map,
                STAND_OFF_DISTANCE_M,
            )
            return "success"
        except Exception:
            return "failure"


class NavigatePoseState(smach.State):
    """导航到 userdata 中保存的目标位姿。"""

    def __init__(self, pose_key):
        smach.State.__init__(self, outcomes=["success", "failure"], input_keys=[pose_key])
        self.pose_key = pose_key

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(NAV_ACTION_NAME, RestaurantNavAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        pose = getattr(userdata, self.pose_key)
        for _ in range(2):
            client.send_goal(build_nav_goal_from_pose(pose))
            client.wait_for_result()
            result = client.get_result()
            if result is not None and result.success:
                return "success"
        return "failure"


class NavigateNameState(smach.State):
    """导航到固定命名航点。"""

    def __init__(self, target_name):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.target_name = target_name

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(NAV_ACTION_NAME, RestaurantNavAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        for _ in range(2):
            client.send_goal(build_nav_goal_from_name(self.target_name))
            client.wait_for_result()
            result = client.get_result()
            if result is not None and result.success:
                return "success"
        return "failure"


class AlignAndTakeOrderState(smach.State):
    """桌边微调并接单。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            output_keys=["service_pose_map", "order_text"],
        )

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(INTERACTION_ACTION_NAME, AlignAndTakeOrderAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        goal = AlignAndTakeOrderGoal(
            ask_text=ASK_ORDER_TEXT,
            max_rounds=ORDER_MAX_ROUNDS,
            align_timeout_sec=ALIGN_TIMEOUT_SEC,
            dialog_timeout_sec=DIALOG_TIMEOUT_SEC,
        )
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        if result is None or not result.success:
            return "failure"

        userdata.service_pose_map = result.service_pose_map
        userdata.order_text = result.order_text
        return "success"


class SpeakOrderAtBarState(smach.State):
    """在吧台直接复述完整订单。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"], input_keys=["order_text"])

    def execute(self, userdata):
        try:
            say_once(ORDER_REPEAT_PREFIX + userdata.order_text)
            return "success"
        except Exception:
            return "failure"


class SkipGraspState(smach.State):
    """抓取占位状态：当前直接跳过。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success"])

    def execute(self, userdata):
        return "success"


class FinalDeliverSpeakState(smach.State):
    """回到顾客后固定播报结束语。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"])

    def execute(self, userdata):
        try:
            say_once(FINAL_DELIVER_TEXT)
            return "success"
        except Exception:
            return "failure"


def main():
    """状态机入口。"""
    rospy.init_node("restaurant_smach")
    sm = smach.StateMachine(outcomes=["task_success", "task_failure"])

    with sm:
        smach.StateMachine.add(
            "FIND_WAVING_CUSTOMER",
            FindWavingCustomerState(),
            transitions={"success": "BUILD_CUSTOMER_NAV_POSE", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "BUILD_CUSTOMER_NAV_POSE",
            BuildCustomerNavPoseState(),
            transitions={"success": "NAV_TO_CUSTOMER", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "NAV_TO_CUSTOMER",
            NavigatePoseState("customer_nav_pose"),
            transitions={"success": "ALIGN_AND_TAKE_ORDER", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "ALIGN_AND_TAKE_ORDER",
            AlignAndTakeOrderState(),
            transitions={"success": "NAV_TO_BAR", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "NAV_TO_BAR",
            NavigateNameState(BAR_TARGET_NAME),
            transitions={"success": "SPEAK_ORDER_AT_BAR", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "SPEAK_ORDER_AT_BAR",
            SpeakOrderAtBarState(),
            transitions={"success": "SKIP_GRASP_PLACEHOLDER", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "SKIP_GRASP_PLACEHOLDER",
            SkipGraspState(),
            transitions={"success": "NAV_BACK_TO_CUSTOMER"},
        )
        smach.StateMachine.add(
            "NAV_BACK_TO_CUSTOMER",
            NavigatePoseState("service_pose_map"),
            transitions={"success": "FINAL_DELIVER_SPEAK", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "FINAL_DELIVER_SPEAK",
            FinalDeliverSpeakState(),
            transitions={"success": "task_success", "failure": "task_failure"},
        )

    introspection_server = smach_ros.IntrospectionServer("restaurant_smach_view", sm, "/RESTAURANT_SM")
    introspection_server.start()
    outcome = sm.execute()
    rospy.loginfo("restaurant smach finished with outcome: %s", outcome)
    introspection_server.stop()


if __name__ == "__main__":
    main()
