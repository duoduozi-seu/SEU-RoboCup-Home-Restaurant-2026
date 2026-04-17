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
5. 每服务完一人后，回到吧台初始坐标，并在 PID 阶段恢复初始朝向。
"""

import sys
from pathlib import Path

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
THIS_FILE = Path(__file__).resolve()
SCRIPT_DIR = str(THIS_FILE.parent)
WORKSPACE_SRC_DIR = str(THIS_FILE.parents[4])
for search_path in (SCRIPT_DIR, WORKSPACE_SRC_DIR):
    if search_path not in sys.path:
        sys.path.insert(0, search_path)

from restaurant_states import build_bar_nav_and_pid_poses, build_customer_nav_pose

# --- 配置常量 ---
WAVE_ACTION_NAME = "find_waving_customer"
NAV_ACTION_NAME = "restaurant_nav"
INTERACTION_ACTION_NAME = "align_and_take_order"
VOICE_SERVICE_NAME = "voice_service"
PIPER_GRIPPER_SERVICE_NAME = "gripper_srv"
PIPER_GO_ZERO_SERVICE_NAME = "go_zero_srv"
PIPER_BLOCK_ARM_SERVICE_NAME = "block_arm"

ODOM_FRAME = "odom"
ROBOT_BASE_FRAME = "base_link"

TARGET_CUSTOMER_COUNT = 2
MAX_CUSTOMERS_PER_SCAN = 2
WAVE_TIMEOUT_SEC = 45.0
WAVE_ANGULAR_SPEED_RAD_S = 0.25
NAV_TIMEOUT_SEC = 120.0
STAND_OFF_DISTANCE_M = 1.0
BAR_NAV_EXTENSION_DISTANCE_M = 1.0
ALIGN_TIMEOUT_SEC = 20.0
DIALOG_TIMEOUT_SEC = 5.0
ORDER_MAX_ROUNDS = 2
ASK_ORDER_TEXT = "Man. What would you like to order you son of bitch?"
ORDER_REPEAT_PREFIX = "The customer ordered "
ORDER_REPEAT_SUFFIX = ", please give me them."
FINAL_DELIVER_TEXT = "I am back. Please take your order. Enjoy your meal. you son of bitch"
POST_GRASP_REQUEST_TEXT = "please give me the rest things"
GRASP_CATEGORY = "bottle"
GRASP_PROFILE_BUNDLE_PATH = ""
GRASP_CONDA_EXE = "conda"
GRASP_DETECT_ENV_NAME = "/home/duo/anaconda3/envs/groundedsam/bin/python"
GRASP_ENV_NAME = "/home/duo/anaconda3/envs/GraspGen/bin/python"
GRASP_CAMERA_SERIAL_NO = ""
GRASP_CAMERA_VISUAL_PRESET = 5
GRASP_CAMERA_WARMUP_FRAMES = 30
CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays", "water", "drink", "milk"]

# 把 0-9 的数字直接映射成英文单词，方便英文语音模型更稳定地读出来。
COUNT_WORD_MAP = {
    0: "zero",
    1: "one",
    2: "two",
    3: "three",
    4: "four",
    5: "five",
    6: "six",
    7: "seven",
    8: "eight",
    9: "nine",
}

_TF_BUFFER = None
_TF_LISTENER = None
_GRASP_RUNTIME = None
_GRASP_BOOT_RUNTIME = None
_GRASP_SHUTDOWN_RUNTIME = None
_GRASP_BOTTLE = None
_GRASP_BOTTLE_CLOSED_SET = None
_PIPER_DELIVER_GET_CONTROLLER = None

DEBUG = False

def load_grasp_api():
    """按需加载抓取模块，避免状态机启动时就绑定整套抓取依赖。"""
    global _GRASP_BOOT_RUNTIME, _GRASP_SHUTDOWN_RUNTIME
    global _GRASP_BOTTLE, _GRASP_BOTTLE_CLOSED_SET
    if (
        _GRASP_BOOT_RUNTIME is not None
        and _GRASP_SHUTDOWN_RUNTIME is not None
        and _GRASP_BOTTLE is not None
        and _GRASP_BOTTLE_CLOSED_SET is not None
    ):
        return

    try:
        from ppc_grasp.scripts.grasp_action import (
            boot_runtime as grasp_boot_runtime,
            shutdown_runtime as grasp_shutdown_runtime,
        )
        from ppc_grasp.scripts.grasp_action.executor import (
            grasp_bottle as grasp_bottle_fn,
            grasp_bottle_closed_set as grasp_bottle_closed_set_fn,
        )
    except Exception as exc:
        raise RuntimeError("failed to import ppc_grasp grasp_action: {}".format(str(exc))) from exc

    _GRASP_BOOT_RUNTIME = grasp_boot_runtime
    _GRASP_SHUTDOWN_RUNTIME = grasp_shutdown_runtime
    _GRASP_BOTTLE = grasp_bottle_fn
    _GRASP_BOTTLE_CLOSED_SET = grasp_bottle_closed_set_fn


def get_grasp_runtime():
    """懒加载并复用抓取 runtime，避免每次抓取都重新启动。"""
    global _GRASP_RUNTIME
    if _GRASP_RUNTIME is not None:
        return _GRASP_RUNTIME

    load_grasp_api()
    _GRASP_RUNTIME = _GRASP_BOOT_RUNTIME(
        profile_bundle_path=GRASP_PROFILE_BUNDLE_PATH,
        conda_exe=GRASP_CONDA_EXE,
        detect_env_name=GRASP_DETECT_ENV_NAME,
        grasp_env_name=GRASP_ENV_NAME,
        camera_serial_no=GRASP_CAMERA_SERIAL_NO,
        camera_visual_preset=GRASP_CAMERA_VISUAL_PRESET,
        camera_warmup_frames=GRASP_CAMERA_WARMUP_FRAMES,
    )
    return _GRASP_RUNTIME


def shutdown_grasp_runtime():
    """统一释放抓取 runtime。"""
    global _GRASP_RUNTIME
    if _GRASP_RUNTIME is None:
        return

    try:
        if _GRASP_SHUTDOWN_RUNTIME is not None:
            _GRASP_SHUTDOWN_RUNTIME(_GRASP_RUNTIME)
    finally:
        _GRASP_RUNTIME = None


def load_piper_deliver_api():
    """按需加载 piper_grab_v2_global 的全局控制器入口。"""
    global _PIPER_DELIVER_GET_CONTROLLER
    if _PIPER_DELIVER_GET_CONTROLLER is not None:
        return

    try:
        from piper_ros.my_piper.scripts.piper_grab_v2_global import get_controller as piper_get_controller
    except Exception as exc:
        raise RuntimeError("failed to import piper_grab_v2_global.get_controller: {}".format(str(exc))) from exc

    _PIPER_DELIVER_GET_CONTROLLER = piper_get_controller


def get_piper_deliver_controller():
    """懒加载并复用送餐阶段使用的 Piper 全局控制器。"""
    load_piper_deliver_api()
    return _PIPER_DELIVER_GET_CONTROLLER(register_service=False)


def normalize_item_name_for_closed_set(item_name):
    """统一订单物品名和闭集类别名，便于精确匹配。"""
    return str(item_name or "").strip().lower().rstrip(".")


def build_closed_set_item_name_set():
    """从可现场调整的闭集列表中生成标准化类别集合。"""
    normalized_names = set()
    for item_name in list(CLOSED_SET_ITEM_NAME_LIST or []):
        normalized_name = normalize_item_name_for_closed_set(item_name)
        if normalized_name:
            normalized_names.add(normalized_name)
    return normalized_names


def build_nav_goal_from_pose(pose: PoseStamped, pid_pose: PoseStamped = None) -> RestaurantNavGoal:
    """把位姿封装成导航 Goal。"""
    goal = RestaurantNavGoal()
    goal.target_mode = RestaurantNavGoal.TARGET_MODE_POSE
    goal.nav_pose = pose
    if pid_pose is not None:
        goal.pid_pose = pid_pose
    goal.timeout_sec = NAV_TIMEOUT_SEC
    return goal


def say_once(text: str):
    """调用现有 TTS 服务播报一句话。"""
    rospy.wait_for_service(VOICE_SERVICE_NAME, timeout=2.0)
    client = rospy.ServiceProxy(VOICE_SERVICE_NAME, VoiceSrv)
    response = client.call(seu_voiceRequest(text=text))
    if not response.state:
        raise RuntimeError("voice service failed: {}".format(response.errormsg))


def say_post_grasp_request_once():
    """抓取状态结束后，统一提示补齐剩余物品。"""
    try:
        say_once(POST_GRASP_REQUEST_TEXT)
    except Exception as exc:
        rospy.logerr("Post-grasp prompt failed: %s", str(exc))


def release_gripper_once(width: float = 0.07, effort: float = 1.0):
    """通过 piper_grab_v2_global 的全局控制器松开夹爪。"""
    controller = get_piper_deliver_controller()
    rospy.loginfo(
        "[DELIVER] release gripper via piper_grab_v2_global | width=%.3f effort=%.2f",
        float(width),
        float(effort),
    )
    if not controller.set_gripper_width(float(width)):
        raise RuntimeError("piper_grab_v2_global failed to open gripper to width={:.3f}".format(float(width)))
    rospy.loginfo("[DELIVER] gripper released by piper_grab_v2_global")


def set_piper_joint_stream_blocked(blocked: bool):
    """暂停/恢复 piper_ctrl_single_node 从 /joint_states 转发控制，避免覆盖服务命令。"""
    from std_srvs.srv import SetBool, SetBoolRequest

    rospy.loginfo(
        "[DELIVER] set %s blocked=%s",
        PIPER_BLOCK_ARM_SERVICE_NAME,
        str(bool(blocked)),
    )
    request = SetBoolRequest()
    request.data = bool(blocked)
    rospy.wait_for_service(PIPER_BLOCK_ARM_SERVICE_NAME, timeout=2.0)
    response = rospy.ServiceProxy(PIPER_BLOCK_ARM_SERVICE_NAME, SetBool).call(request)
    if not response.success and bool(blocked):
        raise RuntimeError("piper block_arm failed: {}".format(response.message))


def move_arm_go_zero_once(is_mit_mode: bool = False):
    """通过 piper_grab_v2_global 的全局控制器回到机械臂零位。"""
    controller = get_piper_deliver_controller()
    rospy.loginfo(
        "[DELIVER] move arm go-zero via piper_grab_v2_global | mit_mode=%s",
        str(bool(is_mit_mode)),
    )
    if not controller.move_to_named_pose("zero"):
        raise RuntimeError("piper_grab_v2_global failed to move arm to named pose 'zero'")
    rospy.loginfo("[DELIVER] arm returned to named pose 'zero' by piper_grab_v2_global")


def convert_count_to_spoken_word(item_count):
    """把数量转换成适合英文播报的单词；仅对 0-9 做简单映射。"""
    try:
        count_value = int(item_count)
    except (TypeError, ValueError):
        return str(item_count)
    return COUNT_WORD_MAP.get(count_value, str(count_value))


def build_bar_order_repeat_text(orderlist, orderlist_num, fallback_order_text=""):
    """把 LLM 抽取出的订单结果拼成吧台前固定复述语句。"""
    repeat_parts = []

    # 这里按“数量英文词 + 物品名”直接拼接，不额外处理量词或单复数。
    for item_name, item_count in zip(list(orderlist or []), list(orderlist_num or [])):
        clean_item_name = str(item_name).strip()
        if not clean_item_name:
            continue
        repeat_parts.append(
            "{} {}".format(
                convert_count_to_spoken_word(item_count),
                clean_item_name,
            )
        )

    if repeat_parts:
        return ORDER_REPEAT_PREFIX + ", ".join(repeat_parts) + ORDER_REPEAT_SUFFIX

    # 如果上游异常导致列表为空，则保留原句兜底，避免这里直接播报失败。
    clean_fallback_order_text = str(fallback_order_text or "").strip()
    if clean_fallback_order_text:
        return ORDER_REPEAT_PREFIX + clean_fallback_order_text

    return ORDER_REPEAT_PREFIX.strip() + ORDER_REPEAT_SUFFIX


def get_tf_buffer():
    """懒加载 TF 监听器。"""
    global _TF_BUFFER, _TF_LISTENER
    if _TF_BUFFER is None:
        _TF_BUFFER = tf2_ros.Buffer()
        _TF_LISTENER = tf2_ros.TransformListener(_TF_BUFFER)
    return _TF_BUFFER


def lookup_robot_pose(timeout_sec=2.0):
    """直接从 TF 读取 odom->base_link 当前位姿。"""
    transform = get_tf_buffer().lookup_transform(
        ODOM_FRAME,
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
    """启动时记录“回吧台”所需的导航点和 PID 点。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            output_keys=[
                "bar_pose_map",
                "bar_pid_pose_map",
                "bar_final_nav_pose_map",
                "bar_final_pid_pose_map",
            ],
        )

    def execute(self, userdata):
        try:
            current_pose = lookup_robot_pose(timeout_sec=2.0)
            bar_nav_pose_map, bar_pid_pose_map = build_bar_nav_and_pid_poses(
                current_pose,
                BAR_NAV_EXTENSION_DISTANCE_M,
            )
            bar_final_nav_pose_map, bar_final_pid_pose_map = build_bar_nav_and_pid_poses(
                current_pose,
                BAR_NAV_EXTENSION_DISTANCE_M,
                pid_reverse_orientation=False,
                nav_reverse_orientation=False,
            )
            userdata.bar_pose_map = bar_nav_pose_map
            userdata.bar_pid_pose_map = bar_pid_pose_map
            userdata.bar_final_nav_pose_map = bar_final_nav_pose_map
            userdata.bar_final_pid_pose_map = bar_final_pid_pose_map
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

            detected_customer_points = person_points[:MAX_CUSTOMERS_PER_SCAN]
            userdata.detected_customer_points = detected_customer_points
            rospy.loginfo(
                "Found %d waving customer(s) in current scan.",
                len(detected_customer_points),
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

        pending_customer_points = detected_customer_points[:remaining_count]
        userdata.pending_customer_points = pending_customer_points
        return "success" if pending_customer_points else "failure"


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

    def __init__(self, pose_key, pid_pose_key=None):
        input_keys = [pose_key]
        if pid_pose_key is not None:
            input_keys.append(pid_pose_key)
        smach.State.__init__(self, outcomes=["success", "failure"], input_keys=input_keys)
        self.pose_key = pose_key
        self.pid_pose_key = pid_pose_key

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(NAV_ACTION_NAME, RestaurantNavAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        pose = getattr(userdata, self.pose_key)
        pid_pose = getattr(userdata, self.pid_pose_key) if self.pid_pose_key is not None else None
        for _ in range(2):
            client.send_goal(build_nav_goal_from_pose(pose, pid_pose=pid_pose))
            client.wait_for_result()
            result = client.get_result()
            if result is not None and result.success:
                return "success"

        rospy.logerr("Navigation to pose '%s' failed.", self.pose_key)
        return "failure"


class NavigateBackToStartPoseState(smach.State):
    """每服务完一人后，回到吧台外延导航点，并保持初始朝向。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["bar_final_nav_pose_map"],
        )

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(NAV_ACTION_NAME, RestaurantNavAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        for _ in range(2):
            client.send_goal(build_nav_goal_from_pose(userdata.bar_final_nav_pose_map))
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
            output_keys=["service_pose_map", "order_text", "orderlist", "orderlist_num"],
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
        userdata.orderlist_num = result.orderlist_num
        return "success"


class SpeakOrderAtBarState(smach.State):
    """在吧台复述订单。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["order_text", "orderlist", "orderlist_num"],
        )

    def execute(self, userdata):
        try:
            repeat_text = build_bar_order_repeat_text(
                getattr(userdata, "orderlist", []),
                getattr(userdata, "orderlist_num", []),
                getattr(userdata, "order_text", ""),
            )
            say_once(repeat_text)
            return "success"
        except Exception:
            return "failure"


class GraspFirstOrderedItemState(smach.State):
    """只抓取订单列表中的第一个物品，并按闭集列表选择抓取入口。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["orderlist", "orderlist_num"],
        )

    def execute(self, userdata):
        CATEGORY = GRASP_CATEGORY
        CLOSED_SET_GDINO_PROMPT = "bottle.lays."

        orderlist = list(getattr(userdata, "orderlist", []) or [])
        orderlist_num = list(getattr(userdata, "orderlist_num", []) or [])
        outcome = "failure"

        try:
            if not orderlist:
                rospy.logwarn("Grasp skipped: orderlist is empty.")
                return outcome

            raw_target_item_name = str(orderlist[0]).strip()
            target_item_name = normalize_item_name_for_closed_set(raw_target_item_name)
            target_item_count = orderlist_num[0] if orderlist_num else 1
            if not target_item_name:
                rospy.logwarn("Grasp skipped: first ordered item is empty.")
                return outcome

            closed_set_item_names = build_closed_set_item_name_set()
            use_closed_set = target_item_name in closed_set_item_names
            grasp_route = "closed_set" if use_closed_set else "grasp_bottle"
            bottle_prompt = CLOSED_SET_GDINO_PROMPT if use_closed_set else target_item_name

            rospy.loginfo(
                "Grasp first ordered item once | route=%s category=%s item=%s normalized_item=%s requested_count=%s gdino_prompt=%s",
                grasp_route,
                CATEGORY,
                raw_target_item_name,
                target_item_name,
                str(target_item_count),
                bottle_prompt,
            )

            try:
                runtime = get_grasp_runtime()
                if use_closed_set:
                    grasp_result = _GRASP_BOTTLE_CLOSED_SET(
                        runtime,
                        target_prompt=target_item_name,
                        gdino_prompt=CLOSED_SET_GDINO_PROMPT,
                    )
                else:
                    grasp_result = _GRASP_BOTTLE(
                        runtime,
                        prompt=bottle_prompt,
                    )
            except Exception as exc:
                rospy.logerr(
                    "Grasp phase exception | route=%s category=%s item=%s error=%s",
                    grasp_route,
                    CATEGORY,
                    target_item_name,
                    str(exc),
                )
                return outcome

            if grasp_result.success:
                rospy.loginfo(
                    "Grasp succeeded | route=%s item=%s status=%s candidate_index=%s",
                    grasp_route,
                    target_item_name,
                    grasp_result.status,
                    str(grasp_result.candidate_index),
                )
                outcome = "success"
                return outcome

            rospy.logwarn(
                "Grasp failed | route=%s item=%s status=%s message=%s",
                grasp_route,
                target_item_name,
                grasp_result.status,
                grasp_result.message,
            )
            return outcome
        finally:
            say_post_grasp_request_once()


class FinalDeliverSpeakState(smach.State):
    """送餐时的结语播报。"""

    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"])

    def execute(self, userdata):
        try:
            say_once(FINAL_DELIVER_TEXT)
            release_gripper_once()
            rospy.sleep(1.0)
            move_arm_go_zero_once()
            rospy.sleep(4.0)
            return "success"
        except Exception as exc:
            rospy.logerr("Final deliver release/go-zero failed: %s", str(exc))
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
        input_keys=[
            "customer_point_map",
            "bar_pose_map",
            "bar_pid_pose_map",
            "bar_final_nav_pose_map",
            "bar_final_pid_pose_map",
        ],
        output_keys=["service_pose_map", "order_text", "orderlist", "orderlist_num"],
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
            NavigatePoseState("bar_pose_map", pid_pose_key="bar_pid_pose_map"),
            transitions={"success": "SPEAK_ORDER_AT_BAR", "failure": "failure"},
        )
        smach.StateMachine.add(
            "SPEAK_ORDER_AT_BAR",
            SpeakOrderAtBarState(),
            transitions={"success": "GRASP_FIRST_ORDERED_ITEM", "failure": "failure"},
        )
        smach.StateMachine.add(
            "GRASP_FIRST_ORDERED_ITEM",
            GraspFirstOrderedItemState(),
            transitions={
                "success": "NAV_BACK_TO_CUSTOMER",
                "failure": "NAV_BACK_TO_CUSTOMER",
            },
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
    sm.userdata.orderlist = []
    sm.userdata.orderlist_num = []

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

    try:
        outcome = sm.execute()
        rospy.loginfo("restaurant smach finished with final outcome: %s", outcome)
    finally:
        try:
            shutdown_grasp_runtime()
        finally:
            introspection_server.stop()


if __name__ == "__main__":
    main()
