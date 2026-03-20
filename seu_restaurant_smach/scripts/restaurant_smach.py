#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
餐厅任务的核心状态机。
按照比赛的标准执行顺序，该脚本组织了以下核心流程：
1. 寻找挥手的顾客 (FIND_WAVING_CUSTOMER)
2. 基于检测点计算导航位姿 (BUILD_CUSTOMER_NAV_POSE)
3. 导航到顾客身旁 (NAV_TO_CUSTOMER)
4. 对准顾客并开启交互点餐 (ALIGN_AND_TAKE_ORDER)
5. 返回吧台进行汇报 (NAV_TO_BAR)
6. 在吧台复述读取到的订单 (SPEAK_ORDER_AT_BAR)
7. 物品抓取（当前占位）(SKIP_GRASP_PLACEHOLDER)
8. 返回顾客处送餐 (NAV_BACK_TO_CUSTOMER)
9. 播报最终完成语 (FINAL_DELIVER_SPEAK)
"""

import os
import sys
import actionlib
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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
# 显式补上当前脚本目录到 sys.path，确保直接运行当前脚本时，
# 同目录下的工具模块（如 restaurant_states.py）也能被稳定导入。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from restaurant_states import build_customer_nav_pose, odometry_to_pose_stamped

# --- 配置常量 ---
WAVE_ACTION_NAME = "find_waving_customer"    # 找寻挥手顾客的 Action 服务名
NAV_ACTION_NAME = "restaurant_nav"           # 餐厅专用导航 Action 服务名
INTERACTION_ACTION_NAME = "align_and_take_order" # 对准并接单 Action 服务名
VOICE_SERVICE_NAME = "voice_service"         # 用于 TTS 播报的消息服务

# 当前导航系统没有直接发布单独的 /robot_pose 话题，
# 机器人在 map 坐标系下的实际位姿是通过 /map_to_baselink（里程计类型）发布的。
ROBOT_POSE_TOPIC = "/map_to_baselink"

BAR_TARGET_NAME = "StartPoint"               # 吧台在地图中的固定点名称
WAVE_TIMEOUT_SEC = 45.0                      # 搜寻挥手的最大尝试时间
WAVE_ANGULAR_SPEED_RAD_S = 0.15              # 旋转搜索时的角速度
NAV_TIMEOUT_SEC = 120.0                      # 每次导航的最大允许时长
STAND_OFF_DISTANCE_M = 1.0                   # 导航点应位于顾客前方 1.0 米
ALIGN_TIMEOUT_SEC = 20.0                     # 互动阶段对准人脸的超时
DIALOG_TIMEOUT_SEC = 20.0                    # 对话识别（ASR）的超时
ORDER_MAX_ROUNDS = 2                         # 询问订单时，若听不清最多重试次数
ASK_ORDER_TEXT = "Hello. What would you like to order?" # 初始问候语内容
ORDER_REPEAT_PREFIX = "The customer ordered: " # 在吧台复述订单时的前导词
FINAL_DELIVER_TEXT = "I am back. Please take your order. Enjoy your meal." # 任务结束播报

# --- 辅助工具函数 ---

def build_nav_goal_from_pose(pose: PoseStamped) -> RestaurantNavGoal:
    """
    辅助函数：将一个 PoseStamped 坐标封装成 RestaurantNav 动作的 Goal。
    """
    goal = RestaurantNavGoal()
    # 模式设为 TARGET_MODE_POSE (0)，告诉执行端使用坐标导航
    goal.target_mode = RestaurantNavGoal.TARGET_MODE_POSE
    goal.nav_pose = pose
    goal.timeout_sec = NAV_TIMEOUT_SEC
    return goal

def build_nav_goal_from_name(target_name: str) -> RestaurantNavGoal:
    """
    辅助函数：将地图中的点名封装成 RestaurantNav 动作的 Goal。
    """
    goal = RestaurantNavGoal()
    # 模式设为 TARGET_MODE_NAME (1)，告诉执行端去预设好的点（如 StartPoint）
    goal.target_mode = RestaurantNavGoal.TARGET_MODE_NAME
    goal.target_name = target_name
    goal.timeout_sec = NAV_TIMEOUT_SEC
    return goal

def say_once(text: str):
    """
    通过 Service 同步调用原有语音服务播报指定文本。
    """
    # 阻塞等待服务出现，防止节点启动顺序问题导致奔溃
    rospy.wait_for_service(VOICE_SERVICE_NAME, timeout=2.0)
    client = rospy.ServiceProxy(VOICE_SERVICE_NAME, VoiceSrv)
    # 构造请求：text 字段填入要说话的内容
    response = client.call(seu_voiceRequest(text=text))
    # 检查返回状态，若失败则抛出运行异常
    if not response.state:
        raise RuntimeError("voice service failed: {}".format(response.errormsg))

# --- 状态类实现 ---

class FindWavingCustomerState(smach.State):
    """
    状态逻辑：调用视觉寻人 Action，返回顾客在 map 坐标系下的坐标。
    """
    def __init__(self):
        # outcomes: 可能的退出结果。success 触发后续逻辑，failure 导致流程终止。
        # output_keys: 向全局状态池写入 'customer_point_map'，存储找人结果。
        smach.State.__init__(self, outcomes=["success", "failure"], output_keys=["customer_point_map"])

    def execute(self, userdata):
        # 初始化 Action 客户端
        client = actionlib.SimpleActionClient(WAVE_ACTION_NAME, FindWavingCustomerAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("[%s] Action server for wave search NOT found!", WAVE_ACTION_NAME)
            return "failure"

        # 尝试 最多 2 次找人过程（应对环境中出现的瞬时干扰）
        for _ in range(2):
            goal = FindWavingCustomerGoal(
                timeout_sec=WAVE_TIMEOUT_SEC,
                angular_speed_rad_s=WAVE_ANGULAR_SPEED_RAD_S,
            )
            client.send_goal(goal)
            client.wait_for_result() # 阻塞直至后端执行完毕或超时
            result = client.get_result()
            
            if result is not None and result.success:
                # 将检测到的顾客三维坐标存入 userdata，供后续 BuildPose 状态读取
                userdata.customer_point_map = result.person_point_map
                rospy.loginfo("Found waving customer at specified point.")
                return "success"
        
        rospy.logwarn("Failed to find waving customer after multiple attempts.")
        return "failure"

class BuildCustomerNavPoseState(smach.State):
    """
    状态逻辑：根据顾客的具体位置，结合当前机器人姿态，计算出一个理想的停车位姿。
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["customer_point_map"], # 输入：上一步找人的结果点
            output_keys=["customer_nav_pose"], # 输出：计算出的导航坐标（PoseStamped）
        )

    def execute(self, userdata):
        try:
            # 1. 从 /map_to_baselink 读取当前时刻机器人的坐标
            robot_odom = rospy.wait_for_message(ROBOT_POSE_TOPIC, Odometry, timeout=2.0)
            # 2. 调用工具函数提取位姿信息
            robot_pose = odometry_to_pose_stamped(robot_odom)
            # 3. 计算逻辑：在机器人与人的连线上取定长（STAND_OFF_DISTANCE_M）位置
            #    这样算出来的点位不但距离合适，且机器人朝向会正对着顾客。
            userdata.customer_nav_pose = build_customer_nav_pose(
                robot_pose,
                userdata.customer_point_map,
                STAND_OFF_DISTANCE_M,
            )
            return "success"
        except Exception as e:
            rospy.logerr("BuildCustomerNavPoseState execution failed: %s", str(e))
            return "failure"

class NavigatePoseState(smach.State):
    """
    状态逻辑：通用的坐标导航状态。
    它会根据构造时传入的 pose_key，动态从 userdata 中读取要去的位姿。
    """
    def __init__(self, pose_key):
        smach.State.__init__(self, outcomes=["success", "failure"], input_keys=[pose_key])
        self.pose_key = pose_key

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(NAV_ACTION_NAME, RestaurantNavAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        # 从 userdata 变量池中取得真正的坐标数据
        pose = getattr(userdata, self.pose_key)
        for _ in range(2):
            # 将坐标组装成导航命令发送
            client.send_goal(build_nav_goal_from_pose(pose))
            client.wait_for_result()
            result = client.get_result()
            # 如果底层导航成功到达（success=True），则该状态成功
            if result is not None and result.success:
                return "success"
        
        rospy.logerr("Navigation to pose '%s' failed.", self.pose_key)
        return "failure"

class NavigateNameState(smach.State):
    """
    状态逻辑：固定的名字点导航（如吧台）。
    """
    def __init__(self, target_name):
        smach.State.__init__(self, outcomes=["success", "failure"])
        self.target_name = target_name

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(NAV_ACTION_NAME, RestaurantNavAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        for _ in range(2):
            # 发送以名字跳转的导航请求
            client.send_goal(build_nav_goal_from_name(self.target_name))
            client.wait_for_result()
            result = client.get_result()
            if result is not None and result.success:
                return "success"
        return "failure"

class AlignAndTakeOrderState(smach.State):
    """
    状态逻辑：顾客桌边的微调对齐与语音点单。
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            output_keys=["service_pose_map", "order_text"], # 输出：完美互动点（存位姿）、识别到的订单文本
        )

    def execute(self, userdata):
        client = actionlib.SimpleActionClient(INTERACTION_ACTION_NAME, AlignAndTakeOrderAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return "failure"

        # 调用后端互动 Action：
        # 它会在当前位置再次用视觉对准人脸中心，对齐底盘方向。
        # 随后，它会自动调用语音服务进行播报（ASK）并录音解析（ASR）。
        goal = AlignAndTakeOrderGoal(
            ask_text=ASK_ORDER_TEXT,
            max_rounds=ORDER_MAX_ROUNDS,
            align_timeout_sec=ALIGN_TIMEOUT_SEC,
            dialog_timeout_sec=DIALOG_TIMEOUT_SEC,
        )
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        
        # 结果校验：如果对话完全失败（如没人回应），则返回 failure
        if result is None or not result.success:
            return "failure"

        # 保存对齐后的具体位姿：回头送货时再用这个点会比之前的计算点更精准。
        userdata.service_pose_map = result.service_pose_map
        # 保存识别出的订单全称
        userdata.order_text = result.order_text
        return "success"

class SpeakOrderAtBarState(smach.State):
    """
    状态逻辑：回到吧台后，把订单“大声”念给服务员。
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"], input_keys=["order_text"])

    def execute(self, userdata):
        try:
            # 拼接台词并调用 TTS 播报
            say_once(ORDER_REPEAT_PREFIX + userdata.order_text)
            return "success"
        except Exception:
            return "failure"

class SkipGraspState(smach.State):
    """
    抓取占位（过渡态）：
    目前的餐厅逻辑中，假设物品已经手动放上来了，这里作为以后机械臂集成时的插入点。
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=["success"])

    def execute(self, userdata):
        rospy.loginfo("Skipped mechanical arm grasp phase.")
        return "success"

class FinalDeliverSpeakState(smach.State):
    """
    状态逻辑：回到顾客身前后，说出“祝您用餐愉快”等结语。
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"])

    def execute(self, userdata):
        try:
            # 调用 TTS
            say_once(FINAL_DELIVER_TEXT)
            return "success"
        except Exception:
            return "failure"

# --- 主入口：构建状态机图谱 ---

def main():
    """
    核心初始化：连接所有分散的状态，形成逻辑闭环。
    """
    # 1. 注册节点
    rospy.init_node("restaurant_smach")
    
    # 2. 创建顶层状态机容器
    # outcomes 为 task_success (全流程结束) 或 task_failure (某个阶段报废)
    sm = smach.StateMachine(outcomes=["task_success", "task_failure"])

    # 使用上下文管理器进行状态节点的添加 (SM Diagram)
    with sm:
        # --- [1] 搜寻阶段 ---
        # 找到正在挥手的顾客，失败则整个大任务报废退出
        smach.StateMachine.add(
            "FIND_WAVING_CUSTOMER",
            FindWavingCustomerState(),
            transitions={"success": "BUILD_CUSTOMER_NAV_POSE", "failure": "task_failure"},
        )
        
        # --- [2] 路径规划 ---
        # 基于第一步的点计算具体的停车横纵坐标和朝向
        smach.StateMachine.add(
            "BUILD_CUSTOMER_NAV_POSE",
            BuildCustomerNavPoseState(),
            transitions={"success": "NAV_TO_CUSTOMER", "failure": "task_failure"},
        )
        
        # --- [3] 接近顾客 ---
        # 第一次主导航，去刚才算出来的位姿
        smach.StateMachine.add(
            "NAV_TO_CUSTOMER",
            NavigatePoseState("customer_nav_pose"),
            transitions={"success": "ALIGN_AND_TAKE_ORDER", "failure": "task_failure"},
        )
        
        # --- [4] 互动获取订单 ---
        # 对准顾客并听取订单信息，成功后会拿到 service_pose_map (对准后的精准位姿)
        smach.StateMachine.add(
            "ALIGN_AND_TAKE_ORDER",
            AlignAndTakeOrderState(),
            transitions={"success": "NAV_TO_BAR", "failure": "task_failure"},
        )
        
        # --- [5] 返回汇报 ---
        # 导航回吧台（StartPoint）
        smach.StateMachine.add(
            "NAV_TO_BAR",
            NavigateNameState(BAR_TARGET_NAME),
            transitions={"success": "SPEAK_ORDER_AT_BAR", "failure": "task_failure"},
        )
        
        # --- [6] 复述订单 ---
        # 在吧台播报识别到的内容
        smach.StateMachine.add(
            "SPEAK_ORDER_AT_BAR",
            SpeakOrderAtBarState(),
            transitions={"success": "SKIP_GRASP_PLACEHOLDER", "failure": "task_failure"},
        )
        
        # --- [7] 物品上车 ---
        # 业务占位点，成功后跳转至返回流程
        smach.StateMachine.add(
            "SKIP_GRASP_PLACEHOLDER",
            SkipGraspState(),
            transitions={"success": "NAV_BACK_TO_CUSTOMER"},
        )
        
        # --- [8] 送餐返回 ---
        # 使用第 4 步存下的精准位姿 service_pose_map 导航回顾客面前
        smach.StateMachine.add(
            "NAV_BACK_TO_CUSTOMER",
            NavigatePoseState("service_pose_map"),
            transitions={"success": "FINAL_DELIVER_SPEAK", "failure": "task_failure"},
        )
        
        # --- [9] 任务结束 ---
        # 播报结语，转入最终 task_success 节点
        smach.StateMachine.add(
            "FINAL_DELIVER_SPEAK",
            FinalDeliverSpeakState(),
            transitions={"success": "task_success", "failure": "task_failure"},
        )

    # --- 后置辅助工具 ---
    # 开启内省服务器，这样在 PC 端运行 `rosrun smach_viewer smach_viewer.py` 
    # 就可以实时看到任务进行到了哪一步，高亮当前活跃状态。
    introspection_server = smach_ros.IntrospectionServer("restaurant_smach_view", sm, "/RESTAURANT_SM")
    introspection_server.start()
    
    # 运行状态机入口
    outcome = sm.execute()
    
    rospy.loginfo("restaurant smach finished with final outcome: %s", outcome)
    introspection_server.stop()

if __name__ == "__main__":
    # 进入 Python main 环境
    main()
