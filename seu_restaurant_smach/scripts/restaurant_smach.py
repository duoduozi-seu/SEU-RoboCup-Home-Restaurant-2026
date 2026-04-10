#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
餐厅任务的双人服务状态机。

顶层状态机只负责三件事：
1. 启动时记录吧台位姿；
2. 维护“待服务顾客队列”和已完成人数；
3. 反复调用“单人服务子状态机”，直到完成两位顾客。

单人服务子状态机负责：
1. 接近当前顾客；
2. 接单；
3. 回吧台播报订单；
4. 回顾客处送餐；
5. 每服务完一人后，回到吧台初始坐标与背对吧台的初始朝向。
"""

import os
import sys

import actionlib
import rospy
import smach
import smach_ros
import tf2_ros

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

# --- 环境补丁 ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from restaurant_states import build_customer_nav_pose, reverse_pose_orientation

# --- 配置常量 ---
WAVE_ACTION_NAME = "find_waving_customer"
NAV_ACTION_NAME = "restaurant_nav"
INTERACTION_ACTION_NAME = "align_and_take_order"
VOICE_SERVICE_NAME = "voice_service"

MAP_FRAME = "map"
ROBOT_BASE_FRAME = "base_link"

TARGET_CUSTOMER_COUNT = 2
MAX_CUSTOMERS_PER_SCAN = 2
WAVE_TIMEOUT_SEC = 45.0
WAVE_ANGULAR_SPEED_RAD_S = 0.25
NAV_TIMEOUT_SEC = 120.0
STAND_OFF_DISTANCE_M = 1.0
ALIGN_TIMEOUT_SEC = 20.0
DIALOG_TIMEOUT_SEC = 20.0
ORDER_MAX_ROUNDS = 2
ASK_ORDER_TEXT = "Hello. What would you like to order?"
ORDER_REPEAT_PREFIX = "The customer ordered: "
FINAL_DELIVER_TEXT = "I am back. Please take your order. Enjoy your meal."

_TF_BUFFER = None
_TF_LISTENER = None


def build_nav_goal_from_pose(pose: PoseStamped) -> RestaurantNavGoal:
    """把位姿封装成导航 Goal。"""
    goal = RestaurantNavGoal()
    goal.target_mode = RestaurantNavGoal.TARGET_MODE_POSE
    goal.nav_pose = pose
    goal.timeout_sec = NAV_TIMEOUT_SEC
    return goal


def say_once(text: str):
    """调用现有 TTS 服务播报一句话。"""
    rospy.wait_for_service(VOICE_SERVICE_NAME, timeout=2.0)
    client = rospy.ServiceProxy(VOICE_SERVICE_NAME, VoiceSrv)
    response = client.call(seu_voiceRequest(text=text))
    if not response.state:
        raise RuntimeError("voice service failed: {}".format(response.errormsg))


def get_tf_buffer():
    """懒加载 TF 监听器。"""
    global _TF_BUFFER, _TF_LISTENER
    if _TF_BUFFER is None:
        _TF_BUFFER = tf2_ros.Buffer()
        _TF_LISTENER = tf2_ros.TransformListener(_TF_BUFFER)
    return _TF_BUFFER


def lookup_robot_pose(timeout_sec=2.0):
    """直接从 TF 读取 map->base_link 当前位姿。"""
    transform = get_tf_buffer().lookup_transform(
        MAP_FRAME,
        ROBOT_BASE_FRAME,
        rospy.Time(0),
        rospy.Duration(timeout_sec),
    )
    pose = PoseStamped()
    pose.header.stamp = transform.header.stamp
    pose.header.frame_id = transform.header.frame_id
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


class CaptureBarPoseState(smach.State):
    """启动时记录“面朝吧台”的位姿。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"], output_keys=["bar_pose_map"])

    def execute(self, userdata):
        try:
            current_pose = lookup_robot_pose(timeout_sec=2.0)
            userdata.bar_pose_map = reverse_pose_orientation(current_pose)
            return "success"
        except Exception as exc:
            rospy.logerr("CaptureBarPoseState execution failed: %s", str(exc))
            return "failure"


class FindWavingCustomersState(smach.State):
    """调用挥手识别 Action，返回本轮扫描得到的顾客点列表。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"], output_keys=["detected_customer_points"])

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(WAVE_ACTION_NAME, FindWavingCustomerAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("[%s] Action server for wave search NOT found!", WAVE_ACTION_NAME)
            return "failure"

        for _ in range(2):
            goal = FindWavingCustomerGoal(
                timeout_sec=WAVE_TIMEOUT_SEC,
                angular_speed_rad_s=WAVE_ANGULAR_SPEED_RAD_S,
            )
            client.send_goal(goal)
            client.wait_for_result()
            result = client.get_result()

            if result is None or not result.success:
                continue

            person_points = list(getattr(result, "person_points_map", []))
            if not person_points:
                continue

            userdata.detected_customer_points = person_points[:MAX_CUSTOMERS_PER_SCAN]
            rospy.loginfo(
                "Found %d waving customer(s) in current scan.",
                len(userdata.detected_customer_points),
            )
            return "success"

        rospy.logwarn("Failed to find waving customer after multiple attempts.")
        return "failure"


class LoadCustomerQueueState(smach.State):
    """把本轮扫描结果装入待服务队列。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["detected_customer_points", "served_customer_count"],
            output_keys=["pending_customer_points"],
        )

    def execute(self, userdata):
        detected_customer_points = list(getattr(userdata, "detected_customer_points", []))
        served_customer_count = int(getattr(userdata, "served_customer_count", 0))
        remaining_count = max(0, TARGET_CUSTOMER_COUNT - served_customer_count)

        if remaining_count <= 0 or not detected_customer_points:
            return "failure"

        userdata.pending_customer_points = detected_customer_points[:remaining_count]
        return "success" if userdata.pending_customer_points else "failure"


class SelectNextCustomerState(smach.State):
    """从待服务队列中取出下一位顾客。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["pending_customer_points"],
            output_keys=["pending_customer_points", "customer_point_map"],
        )

    def execute(self, userdata):
        pending_customer_points = list(getattr(userdata, "pending_customer_points", []))
        if not pending_customer_points:
            return "failure"

        userdata.customer_point_map = pending_customer_points.pop(0)
        userdata.pending_customer_points = pending_customer_points
        return "success"


class BuildCustomerNavPoseState(smach.State):
    """根据顾客绝对点与当前机器人姿态生成导航位姿。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["customer_point_map"],
            output_keys=["customer_nav_pose"],
        )

    def execute(self, userdata):
        try:
            robot_pose = lookup_robot_pose(timeout_sec=2.0)
            userdata.customer_nav_pose = build_customer_nav_pose(
                robot_pose,
                userdata.customer_point_map,
                STAND_OFF_DISTANCE_M,
            )
            return "success"
        except Exception as exc:
            rospy.logerr("BuildCustomerNavPoseState execution failed: %s", str(exc))
            return "failure"


class NavigatePoseState(smach.State):
    """读取指定 userdata 位姿并执行导航。"""

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

        rospy.logerr("Navigation to pose '%s' failed.", self.pose_key)
        return "failure"


class NavigateBackToStartPoseState(smach.State):
    """每服务完一人后，回到吧台初始坐标与背对吧台的朝向。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"], input_keys=["bar_pose_map"])

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(NAV_ACTION_NAME, RestaurantNavAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        start_pose_map = reverse_pose_orientation(userdata.bar_pose_map)
        for _ in range(2):
            client.send_goal(build_nav_goal_from_pose(start_pose_map))
            client.wait_for_result()
            result = client.get_result()
            if result is not None and result.success:
                return "success"

        rospy.logerr("Navigation back to start pose failed.")
        return "failure"


class AlignAndTakeOrderState(smach.State):
    """在顾客桌边对齐并接单。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            output_keys=["service_pose_map", "order_text", "orderlist"],
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
            detail = ""
            if result is not None and getattr(result, "error_msg", ""):
                detail = result.error_msg
            else:
                detail = (client.get_goal_status_text() or "").strip()
            if detail:
                rospy.logerr("Align and take order failed: %s", detail)
            return "failure"

        userdata.service_pose_map = result.service_pose_map
        userdata.order_text = result.order_text
        userdata.orderlist = result.orderlist
        return "success"


class SpeakOrderAtBarState(smach.State):
    """在吧台复述订单。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"], input_keys=["order_text"])

    def execute(self, userdata):
        try:
            say_once(ORDER_REPEAT_PREFIX + userdata.order_text)
            return "success"
        except Exception:
            return "failure"


class SkipGraspState(smach.State):
    """机械臂抓取占位态。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success"])

    def execute(self, userdata):
        rospy.loginfo("Skipped mechanical arm grasp phase.")
        return "success"


class FinalDeliverSpeakState(smach.State):
    """送餐时的结语播报。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"])

    def execute(self, userdata):
        try:
            say_once(FINAL_DELIVER_TEXT)
            return "success"
        except Exception:
            return "failure"


class MarkCustomerServedState(smach.State):
    """每完成一位顾客后，累加已服务人数。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success"],
            input_keys=["served_customer_count"],
            output_keys=["served_customer_count"],
        )

    def execute(self, userdata):
        userdata.served_customer_count = int(getattr(userdata, "served_customer_count", 0)) + 1
        return "success"


class DecideNextStepState(smach.State):
    """根据已完成人数和待服务队列决定下一步。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["serve_next_pending", "find_more", "finished"],
            input_keys=["pending_customer_points", "served_customer_count"],
        )

    def execute(self, userdata):
        served_customer_count = int(getattr(userdata, "served_customer_count", 0))
        pending_customer_points = list(getattr(userdata, "pending_customer_points", []))

        if served_customer_count >= TARGET_CUSTOMER_COUNT:
            return "finished"
        if pending_customer_points:
            return "serve_next_pending"
        return "find_more"


def build_service_one_customer_state_machine():
    """构造“服务一位顾客”的完整子状态机。"""
    service_sm = smach.StateMachine(
        outcomes=["success", "failure"],
        input_keys=["customer_point_map", "bar_pose_map"],
        output_keys=["service_pose_map", "order_text", "orderlist"],
    )

    with service_sm:
        smach.StateMachine.add(
            "BUILD_CUSTOMER_NAV_POSE",
            BuildCustomerNavPoseState(),
            transitions={"success": "NAV_TO_CUSTOMER", "failure": "failure"},
        )
        smach.StateMachine.add(
            "NAV_TO_CUSTOMER",
            NavigatePoseState("customer_nav_pose"),
            transitions={"success": "ALIGN_AND_TAKE_ORDER", "failure": "failure"},
        )
        smach.StateMachine.add(
            "ALIGN_AND_TAKE_ORDER",
            AlignAndTakeOrderState(),
            transitions={"success": "NAV_TO_BAR", "failure": "failure"},
        )
        smach.StateMachine.add(
            "NAV_TO_BAR",
            NavigatePoseState("bar_pose_map"),
            transitions={"success": "SPEAK_ORDER_AT_BAR", "failure": "failure"},
        )
        smach.StateMachine.add(
            "SPEAK_ORDER_AT_BAR",
            SpeakOrderAtBarState(),
            transitions={"success": "SKIP_GRASP_PLACEHOLDER", "failure": "failure"},
        )
        smach.StateMachine.add(
            "SKIP_GRASP_PLACEHOLDER",
            SkipGraspState(),
            transitions={"success": "NAV_BACK_TO_CUSTOMER"},
        )
        smach.StateMachine.add(
            "NAV_BACK_TO_CUSTOMER",
            NavigatePoseState("service_pose_map"),
            transitions={"success": "FINAL_DELIVER_SPEAK", "failure": "failure"},
        )
        smach.StateMachine.add(
            "FINAL_DELIVER_SPEAK",
            FinalDeliverSpeakState(),
            transitions={"success": "NAV_BACK_TO_START_POSE", "failure": "failure"},
        )
        smach.StateMachine.add(
            "NAV_BACK_TO_START_POSE",
            NavigateBackToStartPoseState(),
            transitions={"success": "success", "failure": "failure"},
        )

    return service_sm


def build_top_level_state_machine():
    """构造顶层双人服务状态机。"""
    sm = smach.StateMachine(outcomes=["task_success", "task_failure"])
    sm.userdata.served_customer_count = 0
    sm.userdata.detected_customer_points = []
    sm.userdata.pending_customer_points = []

    service_one_customer_sm = build_service_one_customer_state_machine()

    with sm:
        smach.StateMachine.add(
            "CAPTURE_BAR_POSE",
            CaptureBarPoseState(),
            transitions={"success": "FIND_WAVING_CUSTOMERS", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "FIND_WAVING_CUSTOMERS",
            FindWavingCustomersState(),
            transitions={"success": "LOAD_CUSTOMER_QUEUE", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "LOAD_CUSTOMER_QUEUE",
            LoadCustomerQueueState(),
            transitions={"success": "SELECT_NEXT_CUSTOMER", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "SELECT_NEXT_CUSTOMER",
            SelectNextCustomerState(),
            transitions={"success": "SERVICE_ONE_CUSTOMER", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "SERVICE_ONE_CUSTOMER",
            service_one_customer_sm,
            transitions={"success": "MARK_CUSTOMER_SERVED", "failure": "task_failure"},
        )
        smach.StateMachine.add(
            "MARK_CUSTOMER_SERVED",
            MarkCustomerServedState(),
            transitions={"success": "DECIDE_NEXT_STEP"},
        )
        smach.StateMachine.add(
            "DECIDE_NEXT_STEP",
            DecideNextStepState(),
            transitions={
                "serve_next_pending": "SELECT_NEXT_CUSTOMER",
                "find_more": "FIND_WAVING_CUSTOMERS",
                "finished": "task_success",
            },
        )

    return sm


def main():
    """节点入口。"""
    rospy.init_node("restaurant_smach")

    sm = build_top_level_state_machine()
    introspection_server = smach_ros.IntrospectionServer("restaurant_smach_view", sm, "/RESTAURANT_SM")
    introspection_server.start()

    outcome = sm.execute()
    rospy.loginfo("restaurant smach finished with final outcome: %s", outcome)
    introspection_server.stop()


if __name__ == "__main__":
    main()
