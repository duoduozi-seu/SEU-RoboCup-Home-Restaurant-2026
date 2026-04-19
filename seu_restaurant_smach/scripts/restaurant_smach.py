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

import math
import sys
import time
import contextlib
from pathlib import Path

import actionlib
import rospy
import smach
import smach_ros
import tf.transformations
import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist
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
CMD_VEL_TOPIC = "/cmd_vel"

ODOM_FRAME = "odom"
ROBOT_BASE_FRAME = "base_link"

TARGET_CUSTOMER_COUNT = 2
MAX_CUSTOMERS_PER_SCAN = 2
WAVE_TIMEOUT_SEC = 45.0
WAVE_ANGULAR_SPEED_RAD_S = 0.25
NAV_TIMEOUT_SEC = 120.0
STAND_OFF_DISTANCE_M = 1.0
BAR_NAV_EXTENSION_DISTANCE_M = 0.5
ALIGN_TIMEOUT_SEC = 20.0
DIALOG_TIMEOUT_SEC = 5.0
ORDER_MAX_ROUNDS = 2
ASK_ORDER_TEXT = "What would you like to order, bro"
ORDER_REPEAT_PREFIX = "The customer ordered "
ORDER_REPEAT_SUFFIX = ", please give me them you know M sam"
FINAL_DELIVER_TEXT = "I am back. Please take your order. Enjoy your meal bro."
POST_GRASP_REQUEST_TEXT = "please give me the rest things bro"
GRASP_CATEGORY = "bottle.objects."
GRASP_PROFILE_BUNDLE_PATH = ""
GRASP_CONDA_EXE = "conda"
GRASP_DETECT_ENV_NAME = "/home/duo/anaconda3/envs/groundedsam/bin/python"
GRASP_ENV_NAME = "/home/duo/anaconda3/envs/GraspGen/bin/python"
GRASP_CAMERA_SERIAL_NO = "f0244552"
GRASP_CAMERA_VISUAL_PRESET = 5
GRASP_CAMERA_WARMUP_FRAMES = 30
GRASP_WARM_START_SERVICE_TIMEOUT_SEC = 30.0
GRASP_WARM_START_REQUIRED_SERVICES = (
    "joint_moveit_ctrl_arm",
    "joint_moveit_ctrl_gripper",
)
CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays", "water", "drink", "milk"]

# 抓取旋转寻找参数
# 餐厅任务现在只允许一次最多抓 3 件“实际物品”，不是 3 个名称。
MAX_GRASP_ITEM_COUNT = 3
GRASP_RETRY_YAW_OFFSETS_DEG = [0.0, 15.0, -15.0]
GRASP_RETRY_ANGULAR_SPEED_RAD_S = 0.25
GRASP_RETRY_YAW_TOLERANCE_RAD = math.radians(2.0)
GRASP_RETRY_TIMEOUT_SEC = 8.0
GRASP_ROTATE_CONTROL_HZ = 20.0
# 抓取重试前后位移参数。
GRASP_RETRY_LINEAR_TARGET_OFFSETS_M = [0.0, -0.15, 0.05]
GRASP_RETRY_LINEAR_SPEED_M_S = 0.10
GRASP_RETRY_LINEAR_TOLERANCE_M = 0.01
GRASP_RETRY_LINEAR_CONTROL_HZ = 20.0
# 抓取结束后的统一倒退参数。
GRASP_BACKOFF_DISTANCE_M = 0.5
GRASP_BACKOFF_SPEED_M_S = 0.5
GRASP_BACKOFF_CONTROL_HZ = 20.0
GRASP_BACKOFF_TOLERANCE_M = 0.01

CLOSED_SET_FAILURE_KIND_NO_MATCH = "no_match"
CLOSED_SET_FAILURE_KIND_GRASP_ERROR = "grasp_error"

# 下面 5 组是本任务里写死的机械臂关节常量，后续现场联调只改这里。
# 现有托盘位姿 1：托盘上方的中转位。放入托盘和从托盘取出时都先到这里。
TRAY_ABOVE_JOINTS_RAD = [-1.605, 1.386, -0.908, 0.00, 1.191, -1.538]
# 现有托盘位姿 2：托盘的落放/取件位。放入托盘和从托盘取出都用这个点。
TRAY_PICK_JOINTS_RAD = [-1.605, 1.703, -0.592, 0.0, 0.559, -1.538] #[1.605, 1.703, -0.592, 0.0, 0.559, -1.538]
# 新增托盘位姿 1：另一个托盘上方的中转位。
TRAY_ABOVE_JOINTS_RAD_OTHER = [1.605, 1.386, -0.908, 0.00, 1.191, -1.538]
# 新增托盘位姿 2：另一个托盘的落放/取件位。
TRAY_PICK_JOINTS_RAD_OTHER = [1.605, 1.703, -0.592, 0.0, 0.559, -1.538]
# 位姿 3：最终把物品伸到顾客面前，再松开夹爪。
DELIVER_EXTEND_JOINTS_RAD = [0.0, 1.491, -0.942, 0.0, -0.549, 0.0]

# 夹爪在本状态机里只需要最简单的“张开/闭合”两个固定值。
GRIPPER_OPEN_WIDTH_M = 0.09
GRIPPER_CLOSE_WIDTH_M = 0.0

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
_CMD_VEL_PUBLISHER = None

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


def hot_start_grasp_runtime():
    """在状态机执行前前台阻塞完成抓取 runtime 热启动。"""
    if _GRASP_RUNTIME is not None:
        return _GRASP_RUNTIME

    for service_name in GRASP_WARM_START_REQUIRED_SERVICES:
        rospy.loginfo(
            "Waiting for grasp warm-start dependency: %s (timeout=%.1fs)",
            service_name,
            GRASP_WARM_START_SERVICE_TIMEOUT_SEC,
        )
        try:
            rospy.wait_for_service(
                service_name,
                timeout=GRASP_WARM_START_SERVICE_TIMEOUT_SEC,
            )
        except Exception as exc:
            raise RuntimeError(
                "grasp warm-start blocked: service '{}' not available within {:.1f}s: {}".format(
                    service_name,
                    GRASP_WARM_START_SERVICE_TIMEOUT_SEC,
                    str(exc),
                )
            ) from exc

    rospy.loginfo("Starting grasp runtime warm-start")
    runtime = get_grasp_runtime()
    rospy.loginfo("Grasp runtime warm-start completed")
    return runtime


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


def build_closed_set_item_name_list():
    """从可现场调整的闭集列表中生成标准化类别列表，并保留原顺序。"""
    normalized_names = []
    for item_name in list(CLOSED_SET_ITEM_NAME_LIST or []):
        normalized_name = normalize_item_name_for_closed_set(item_name)
        if normalized_name and normalized_name not in normalized_names:
            normalized_names.append(normalized_name)
    return normalized_names


def build_closed_set_item_name_set():
    """从可现场调整的闭集列表中生成标准化类别集合。"""
    return set(build_closed_set_item_name_list())


def build_closed_set_gdino_prompt():
    """把闭集类别列表拼成 gdino prompt。"""
    closed_set_item_names = build_closed_set_item_name_list()
    if not closed_set_item_names:
        return ""
    return ".".join(closed_set_item_names) + "."


def build_target_item_name_list_from_order(orderlist, orderlist_num):
    """把订单按数量展开成实际抓取序列，并截断到当前任务允许的最大抓取件数。"""
    target_item_name_list = []
    order_name_list = list(orderlist or [])
    order_count_list = list(orderlist_num or [])

    for item_index, raw_target_item_name in enumerate(order_name_list):
        target_item_name = normalize_item_name_for_closed_set(raw_target_item_name)
        if not target_item_name:
            continue

        # 上游数量异常时，默认按 1 件处理，避免这里把可抓的物品直接丢掉。
        try:
            repeat_count = int(order_count_list[item_index]) if item_index < len(order_count_list) else 1
        except (TypeError, ValueError):
            repeat_count = 1

        for _ in range(max(0, repeat_count)):
            target_item_name_list.append(target_item_name)
            if len(target_item_name_list) >= MAX_GRASP_ITEM_COUNT:
                return target_item_name_list

    return target_item_name_list


def get_cmd_vel_publisher():
    """懒加载底盘速度发布器。"""
    global _CMD_VEL_PUBLISHER
    if _CMD_VEL_PUBLISHER is None:
        _CMD_VEL_PUBLISHER = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    return _CMD_VEL_PUBLISHER


def stop_base_motion():
    """统一发送零速，避免底盘残留速度。"""
    cmd_vel_publisher = get_cmd_vel_publisher()
    for _ in range(3):
        cmd_vel_publisher.publish(Twist())
        rospy.sleep(0.03)


def normalize_angle_rad(angle_rad):
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def pose_to_yaw(pose):
    """从 PoseStamped 中提取平面 yaw。"""
    quaternion = [
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ]
    return tf.transformations.euler_from_quaternion(quaternion)[2]


def rotate_base_to_relative_yaw(initial_yaw, relative_yaw_offset_rad):
    """相对单个物品开始抓取时的初始朝向，原地旋转到底盘目标角。"""
    target_yaw = normalize_angle_rad(float(initial_yaw) + float(relative_yaw_offset_rad))
    cmd_vel_publisher = get_cmd_vel_publisher()
    rate = rospy.Rate(GRASP_ROTATE_CONTROL_HZ)
    deadline = time.monotonic() + GRASP_RETRY_TIMEOUT_SEC

    try:
        while True:
            try:
                current_yaw = pose_to_yaw(lookup_robot_pose(timeout_sec=0.5))
            except Exception:
                return False

            yaw_error = normalize_angle_rad(target_yaw - current_yaw)
            if abs(yaw_error) <= GRASP_RETRY_YAW_TOLERANCE_RAD:
                return True
            if time.monotonic() >= deadline:
                return False

            twist = Twist()
            twist.angular.z = math.copysign(GRASP_RETRY_ANGULAR_SPEED_RAD_S, yaw_error)
            cmd_vel_publisher.publish(twist)
            rate.sleep()
    finally:
        stop_base_motion()


def project_pose_forward_offset(reference_pose, current_pose):
    """把当前位置投影到参考位姿朝向上，返回 base_link 前进方向位移。"""
    reference_x = float(reference_pose.pose.position.x)
    reference_y = float(reference_pose.pose.position.y)
    reference_yaw = pose_to_yaw(reference_pose)
    current_x = float(current_pose.pose.position.x)
    current_y = float(current_pose.pose.position.y)
    dx = current_x - reference_x
    dy = current_y - reference_y
    return dx * math.cos(reference_yaw) + dy * math.sin(reference_yaw)


def move_base_to_reference_forward_offset(
    reference_pose,
    target_forward_offset_m,
    *,
    linear_speed_m_s,
    tolerance_m,
    control_hz,
    stage_label,
):
    """基于 odom/tf 闭环，把底盘移动到参考位姿前向投影的目标偏移位置。"""
    if reference_pose is None:
        return False

    speed_m_s = abs(float(linear_speed_m_s))
    if speed_m_s <= 0.0:
        return False

    target_offset_m = float(target_forward_offset_m)
    cmd_vel_publisher = get_cmd_vel_publisher()
    rate = rospy.Rate(float(control_hz))

    try:
        current_pose = lookup_robot_pose(timeout_sec=0.5)
    except Exception:
        rospy.logwarn("[%s] current odom pose unavailable before base move", stage_label)
        return False

    initial_error = target_offset_m - project_pose_forward_offset(reference_pose, current_pose)
    deadline = time.monotonic() + max(2.0, abs(initial_error) / speed_m_s + 2.0)

    try:
        while not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                rospy.logwarn("[%s] timeout while moving base to forward offset %.3fm", stage_label, target_offset_m)
                return False

            try:
                current_pose = lookup_robot_pose(timeout_sec=0.5)
            except Exception:
                rospy.logwarn("[%s] waiting odom/tf during base move...", stage_label)
                rate.sleep()
                continue

            current_offset = project_pose_forward_offset(reference_pose, current_pose)
            forward_error = target_offset_m - current_offset
            if abs(forward_error) <= float(tolerance_m):
                return True

            twist = Twist()
            twist.linear.x = math.copysign(speed_m_s, forward_error)
            cmd_vel_publisher.publish(twist)
            rate.sleep()
    finally:
        stop_base_motion()


def extract_closed_set_grasp_failure_kind(grasp_result):
    """把闭集抓取失败最小分成 no_match 和 grasp_error。"""
    if getattr(grasp_result, "success", False):
        return ""

    result_status = str(getattr(grasp_result, "status", "") or "").strip().lower()
    detect_result = getattr(grasp_result, "detect_result", None)
    detect_status = ""
    if isinstance(detect_result, dict):
        detect_status = str(detect_result.get("status", "") or "").strip().lower()

    if result_status == "closed_set_no_match" or detect_status == "closed_set_no_match":
        return CLOSED_SET_FAILURE_KIND_NO_MATCH
    return CLOSED_SET_FAILURE_KIND_GRASP_ERROR


def log_closed_set_grasp_failure(target_item_name, failure_kind, grasp_result=None, exc=None):
    """统一打印闭集抓取失败类型，方便现场看日志。"""
    detect_status = ""
    message = ""
    if grasp_result is not None:
        detect_result = getattr(grasp_result, "detect_result", None)
        if isinstance(detect_result, dict):
            detect_status = str(detect_result.get("status", "") or "").strip()
        message = str(getattr(grasp_result, "message", "") or "").strip()
    if exc is not None:
        message = str(exc)

    rospy.logwarn(
        "Closed-set grasp failed | item=%s kind=%s detect_status=%s message=%s",
        target_item_name,
        failure_kind or "unknown",
        detect_status,
        message,
    )


def drive_base_backward_after_grasp():
    """抓取状态结束前，按 odom/tf 闭环短距离后退给后续动作留空间。"""
    if GRASP_BACKOFF_DISTANCE_M <= 0.0 or GRASP_BACKOFF_SPEED_M_S <= 0.0:
        return False

    try:
        reference_pose = lookup_robot_pose(timeout_sec=2.0)
    except Exception as exc:
        rospy.logwarn("Grasp backoff skipped: failed to get odom/tf pose: %s", str(exc))
        return False

    return move_base_to_reference_forward_offset(
        reference_pose,
        -float(GRASP_BACKOFF_DISTANCE_M),
        linear_speed_m_s=GRASP_BACKOFF_SPEED_M_S,
        tolerance_m=GRASP_BACKOFF_TOLERANCE_M,
        control_hz=GRASP_BACKOFF_CONTROL_HZ,
        stage_label="grasp_backoff",
    )


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


def set_gripper_width_once(width: float, controller=None):
    """通过全局控制器设置一次夹爪开度。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    rospy.loginfo("[DELIVER] set gripper width via piper_grab_v2_global | width=%.3f", float(width))
    if not controller.set_gripper_width(float(width)):
        raise RuntimeError("piper_grab_v2_global failed to set gripper width to {:.3f}".format(float(width)))
    rospy.loginfo("[DELIVER] gripper width set by piper_grab_v2_global")


def release_gripper_once(width: float = GRIPPER_OPEN_WIDTH_M, effort: float = 1.0, controller=None):
    """通过 piper_grab_v2_global 的全局控制器松开夹爪。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    rospy.loginfo(
        "[DELIVER] release gripper via piper_grab_v2_global | width=%.3f effort=%.2f",
        float(width),
        float(effort),
    )
    set_gripper_width_once(float(width), controller=controller)


def move_arm_to_named_pose_once(target_name: str, controller=None):
    """通过全局控制器移动到已有命名位姿。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    rospy.loginfo("[DELIVER] move arm to named pose via piper_grab_v2_global | pose=%s", str(target_name))
    if not controller.move_to_named_pose(str(target_name)):
        raise RuntimeError("piper_grab_v2_global failed to move arm to named pose '{}'".format(str(target_name)))
    rospy.loginfo("[DELIVER] arm reached named pose '%s' by piper_grab_v2_global", str(target_name))


def move_arm_to_joint_positions_once(target_joints_rad, description="", controller=None):
    """通过全局控制器把机械臂移动到一组固定关节角。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    target_joints = [float(value) for value in list(target_joints_rad or [])]
    if len(target_joints) != 6:
        raise ValueError("target_joints_rad must contain 6 values, got {}".format(len(target_joints)))

    log_desc = str(description or "unnamed_joint_target")
    rospy.loginfo("[DELIVER] move arm to fixed joints | desc=%s joints=%s", log_desc, target_joints)
    lock = getattr(controller, "_motion_lock", None)
    ctx = lock if lock is not None else contextlib.nullcontext()
    with ctx:
        controller.arm.set_joint_value_target(target_joints)
        arm_ok = bool(controller.arm.go(wait=True))
        with contextlib.suppress(Exception):
            controller.arm.stop()

    if not arm_ok:
        raise RuntimeError("piper_grab_v2_global failed to move arm to fixed joints '{}'".format(log_desc))
    rospy.loginfo("[DELIVER] arm reached fixed joints | desc=%s", log_desc)


def move_arm_go_zero_once(is_mit_mode: bool = False, controller=None):
    """通过 piper_grab_v2_global 的全局控制器回到机械臂零位。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    rospy.loginfo(
        "[DELIVER] move arm go-zero via piper_grab_v2_global | mit_mode=%s",
        str(bool(is_mit_mode)),
    )
    move_arm_to_named_pose_once("zero", controller=controller)


def count_delivery_item_count_from_order_num(orderlist_num):
    """交付阶段只按数量列表求总件数，并截断到本任务允许的最大值。"""
    total_item_count = 0
    for raw_item_count in list(orderlist_num or []):
        try:
            item_count = int(raw_item_count)
        except (TypeError, ValueError):
            continue
        total_item_count += max(0, item_count)
        if total_item_count >= MAX_GRASP_ITEM_COUNT:
            return MAX_GRASP_ITEM_COUNT
    return total_item_count


def get_post_grasp_storage_target(target_item_count, successful_grasp_count):
    """根据本轮目标总件数和当前成功抓取序号，决定这件物品放哪里。"""
    capped_target_count = max(0, min(int(target_item_count), MAX_GRASP_ITEM_COUNT))
    if successful_grasp_count < 1 or successful_grasp_count > capped_target_count:
        return "skip"
    if capped_target_count <= 1:
        return "gripper"
    if capped_target_count == 2:
        return "main_tray" if successful_grasp_count == 1 else "gripper"
    if successful_grasp_count == 1:
        return "main_tray"
    if successful_grasp_count == 2:
        return "other_tray"
    return "gripper"


def place_item_on_tray_once(tray_above_joints_rad, tray_pick_joints_rad, description_prefix, controller=None):
    """把当前夹爪里拿着的物品放入指定托盘，然后回到 zero。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    prefix = str(description_prefix or "tray")
    # 放托盘固定序列：位姿1 -> 位姿2 -> 松夹爪 -> 位姿1 -> zero。
    move_arm_to_joint_positions_once(
        tray_above_joints_rad,
        description="{}_pose1_before_place".format(prefix),
        controller=controller,
    )
    move_arm_to_joint_positions_once(
        tray_pick_joints_rad,
        description="{}_pose2_place_down".format(prefix),
        controller=controller,
    )
    release_gripper_once(controller=controller)
    move_arm_to_joint_positions_once(
        tray_above_joints_rad,
        description="{}_pose1_after_place".format(prefix),
        controller=controller,
    )
    move_arm_go_zero_once(controller=controller)


def deliver_item_from_tray_once(tray_above_joints_rad, tray_pick_joints_rad, description_prefix, controller=None):
    """从指定托盘取出一件物品并送到顾客面前。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    prefix = str(description_prefix or "tray")
    # 从托盘取出固定序列：位姿1 -> 张开夹爪 -> 位姿2 -> 闭夹爪 -> 位姿1 -> 位姿3 -> 松夹爪。
    move_arm_to_joint_positions_once(
        tray_above_joints_rad,
        description="{}_pose1_before_pick".format(prefix),
        controller=controller,
    )
    release_gripper_once(controller=controller)
    move_arm_to_joint_positions_once(
        tray_pick_joints_rad,
        description="{}_pose2_pick_down".format(prefix),
        controller=controller,
    )
    set_gripper_width_once(GRIPPER_CLOSE_WIDTH_M, controller=controller)
    move_arm_to_joint_positions_once(
        tray_above_joints_rad,
        description="{}_pose1_after_pick".format(prefix),
        controller=controller,
    )
    move_arm_to_joint_positions_once(
        DELIVER_EXTEND_JOINTS_RAD,
        description="{}_pose3_deliver".format(prefix),
        controller=controller,
    )
    release_gripper_once(controller=controller)
    rospy.sleep(1.0)


def deliver_item_from_gripper_once(controller=None):
    """直接把当前夹爪里拿着的物品送给顾客，然后回到 zero。"""
    controller = get_piper_deliver_controller() if controller is None else controller
    # 夹爪直送固定序列：位姿3 -> 松夹爪 -> zero。
    move_arm_to_joint_positions_once(
        DELIVER_EXTEND_JOINTS_RAD,
        description="gripper_direct_pose3_deliver",
        controller=controller,
    )
    release_gripper_once(controller=controller)
    rospy.sleep(1.0)
    move_arm_go_zero_once(controller=controller)


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
            closed_set_item_name_list=build_closed_set_item_name_list(),
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
    """兼容旧类名；按实际件数顺序尝试闭集抓取最多三个物品。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["orderlist", "orderlist_num"],
        )

    def execute(self, userdata):
        orderlist = list(getattr(userdata, "orderlist", []) or [])
        orderlist_num = list(getattr(userdata, "orderlist_num", []) or [])
        target_item_name_list = build_target_item_name_list_from_order(orderlist, orderlist_num)
        target_item_count = len(target_item_name_list)

        try:
            if not target_item_name_list:
                return "success"

            gdino_prompt = build_closed_set_gdino_prompt()
            if not gdino_prompt:
                for target_item_name in target_item_name_list:
                    rospy.logwarn("Closed-set grasp failed: %s", target_item_name)
                return "success"

            runtime = get_grasp_runtime()
            successful_grasp_count = 0
            for target_item_name in target_item_name_list:
                initial_pose = None
                initial_yaw = None
                item_grasp_success = False
                first_failure_kind = ""

                def run_closed_set_grasp_attempt():
                    try:
                        grasp_result = _GRASP_BOTTLE_CLOSED_SET(
                            runtime,
                            target_prompt=target_item_name,
                            gdino_prompt=gdino_prompt,
                        )
                    except Exception as exc:
                        failure_kind = CLOSED_SET_FAILURE_KIND_GRASP_ERROR
                        log_closed_set_grasp_failure(
                            target_item_name,
                            failure_kind,
                            grasp_result=None,
                            exc=exc,
                        )
                        return False, failure_kind

                    if getattr(grasp_result, "success", False):
                        return True, ""

                    failure_kind = extract_closed_set_grasp_failure_kind(grasp_result)
                    log_closed_set_grasp_failure(
                        target_item_name,
                        failure_kind,
                        grasp_result=grasp_result,
                    )
                    return False, failure_kind

                try:
                    try:
                        initial_pose = lookup_robot_pose(timeout_sec=2.0)
                        initial_yaw = pose_to_yaw(initial_pose)
                    except Exception as exc:
                        rospy.logwarn(
                            "Failed to capture initial base pose before grasp retry | item=%s error=%s",
                            target_item_name,
                            str(exc),
                        )
                        initial_pose = None
                        initial_yaw = None

                    item_grasp_success, first_failure_kind = run_closed_set_grasp_attempt()
                    if item_grasp_success:
                        pass
                    elif first_failure_kind == CLOSED_SET_FAILURE_KIND_NO_MATCH and initial_yaw is not None:
                        for offset_deg in GRASP_RETRY_YAW_OFFSETS_DEG[1:]:
                            try:
                                rotate_base_to_relative_yaw(initial_yaw, math.radians(offset_deg))
                            except Exception:
                                pass

                            item_grasp_success, _ = run_closed_set_grasp_attempt()
                            if item_grasp_success:
                                break
                    elif (
                        first_failure_kind == CLOSED_SET_FAILURE_KIND_GRASP_ERROR
                        and initial_pose is not None
                    ):
                        for target_offset_m in GRASP_RETRY_LINEAR_TARGET_OFFSETS_M[1:]:
                            try:
                                move_base_to_reference_forward_offset(
                                    initial_pose,
                                    float(target_offset_m),
                                    linear_speed_m_s=GRASP_RETRY_LINEAR_SPEED_M_S,
                                    tolerance_m=GRASP_RETRY_LINEAR_TOLERANCE_M,
                                    control_hz=GRASP_RETRY_LINEAR_CONTROL_HZ,
                                    stage_label="grasp_retry_linear",
                                )
                            except Exception:
                                pass

                            item_grasp_success, _ = run_closed_set_grasp_attempt()
                            if item_grasp_success:
                                break
                except Exception as exc:
                    log_closed_set_grasp_failure(
                        target_item_name,
                        first_failure_kind or CLOSED_SET_FAILURE_KIND_GRASP_ERROR,
                        grasp_result=None,
                        exc=exc,
                    )
                finally:
                    if initial_yaw is not None:
                        try:
                            rotate_base_to_relative_yaw(initial_yaw, 0.0)
                        except Exception:
                            pass
                    if initial_pose is not None:
                        try:
                            move_base_to_reference_forward_offset(
                                initial_pose,
                                0.0,
                                linear_speed_m_s=GRASP_RETRY_LINEAR_SPEED_M_S,
                                tolerance_m=GRASP_RETRY_LINEAR_TOLERANCE_M,
                                control_hz=GRASP_RETRY_LINEAR_CONTROL_HZ,
                                stage_label="grasp_retry_restore",
                            )
                        except Exception:
                            pass

                if not item_grasp_success:
                    continue

                successful_grasp_count += 1
                storage_target = get_post_grasp_storage_target(target_item_count, successful_grasp_count)

                # 这次任务不再维护“夹爪/托盘是否有物品”的内部标记，
                # 只按本轮目标总件数把成功抓到的物品落到固定位置。
                if storage_target == "main_tray":
                    place_item_on_tray_once(
                        TRAY_ABOVE_JOINTS_RAD,
                        TRAY_PICK_JOINTS_RAD,
                        description_prefix="main_tray",
                        controller=runtime.controller,
                    )
                    continue

                if storage_target == "other_tray":
                    place_item_on_tray_once(
                        TRAY_ABOVE_JOINTS_RAD_OTHER,
                        TRAY_PICK_JOINTS_RAD_OTHER,
                        description_prefix="other_tray",
                        controller=runtime.controller,
                    )
                    continue

                # 需要直接交付给顾客的那一件保留在夹爪里即可。
                if storage_target == "gripper":
                    break
        except Exception as exc:
            rospy.logwarn("Closed-set grasp failed: %s", str(exc))
        finally:
            say_post_grasp_request_once()
            drive_base_backward_after_grasp()

        return "success"


class FinalDeliverSpeakState(smach.State):
    """送餐时的结语播报。"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["orderlist_num"],
        )

    def execute(self, userdata):
        try:
            say_once(FINAL_DELIVER_TEXT)
            controller = get_piper_deliver_controller()
            deliver_item_count = count_delivery_item_count_from_order_num(
                getattr(userdata, "orderlist_num", []),
            )

            # 交付阶段不再维护真实占用状态，只按数量列表总件数决定固定动作序列。
            if deliver_item_count >= 1:
                deliver_item_from_gripper_once(controller=controller)

            if deliver_item_count >= 2:
                deliver_item_from_tray_once(
                    TRAY_ABOVE_JOINTS_RAD,
                    TRAY_PICK_JOINTS_RAD,
                    description_prefix="main_tray",
                    controller=controller,
                )

            if deliver_item_count >= 3:
                deliver_item_from_tray_once(
                    TRAY_ABOVE_JOINTS_RAD_OTHER,
                    TRAY_PICK_JOINTS_RAD_OTHER,
                    description_prefix="other_tray",
                    controller=controller,
                )

            if deliver_item_count >= 2:
                move_arm_go_zero_once(controller=controller)

            if deliver_item_count > 0:
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
    introspection_server = None

    try:
        hot_start_grasp_runtime()
        sm = build_top_level_state_machine()
        introspection_server = smach_ros.IntrospectionServer("restaurant_smach_view", sm, "/RESTAURANT_SM")
        introspection_server.start()
        outcome = sm.execute()
        rospy.loginfo("restaurant smach finished with final outcome: %s", outcome)
    finally:
        try:
            shutdown_grasp_runtime()
        finally:
            if introspection_server is not None:
                introspection_server.stop()


if __name__ == "__main__":
    main()
