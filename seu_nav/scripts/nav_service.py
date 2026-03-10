#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
seu_nav 导航服务节点 (2026版)

功能概述：
    提供 ROS Service 接口 `nav_service`，接收目标地点名称（字符串），
    通过 catkin_robocup_2 提供的 `task_nav` Action Server 执行导航，
    返回导航结果（成功/失败 + 错误信息）。

航点管理：
    航点坐标从 YAML 文件中加载，YAML 格式与 catkin_robocup_2 中的
    waypoints.yaml 保持一致。

使用方式：
    rosrun seu_nav nav_service.py _waypoints_file:=/path/to/waypoints.yaml
"""

import os
import rospy
import yaml
import actionlib

from geometry_msgs.msg import PoseStamped
from tracer_nav.msg import TaskNavAction, TaskNavGoal
from seu_nav.srv import seu_nav, seu_navResponse

# ============================================================
# 常量定义
# ============================================================
SERVICE_NAME = 'nav_service'          # ROS 服务名称，上层状态机通过此名称调用
ACTION_NAME = 'task_nav'              # catkin_robocup_2 提供的导航 Action 名称
DEFAULT_TIMEOUT_SEC = 120             # 默认导航超时时间（秒）
ACTION_CONNECT_TIMEOUT_SEC = 10       # 等待 Action Server 连接的超时时间（秒）

# 硬编码航点文件路径，相对于当前脚本位置
WAYPOINTS_YAML_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'waypoints.yaml'
)


class NavServiceNode:
    """导航服务节点类。

    负责：
        1. 从 YAML 文件加载航点字典
        2. 建立与 task_nav Action Server 的连接
        3. 暴露 nav_service ROS Service，将地点名称转换为导航目标
    """

    def __init__(self):
        """初始化节点：加载航点、连接 Action Server、注册 Service。"""
        rospy.init_node('nav_service_node')

        # --------------------------------------------------
        # 1. 加载航点 YAML 文件
        # --------------------------------------------------
        # 使用与作者 waypoint_navigator 对齐的方式加载数据
        # self.waypoints_data 将直接存储 yaml 的原始数据结构
        self.waypoints_data = self._load_waypoints(WAYPOINTS_YAML_PATH)

        # 导航超时时间（秒），可通过 ~timeout 参数配置
        self.timeout = rospy.get_param('~timeout', DEFAULT_TIMEOUT_SEC)

        # --------------------------------------------------
        # 2. 连接 task_nav Action Server
        # --------------------------------------------------
        self.nav_client = actionlib.SimpleActionClient(ACTION_NAME, TaskNavAction)
        rospy.loginfo('[seu_nav] 正在等待 task_nav Action Server ...')
        connected = self.nav_client.wait_for_server(
            rospy.Duration(ACTION_CONNECT_TIMEOUT_SEC)
        )
        if connected:
            rospy.loginfo('[seu_nav] 已连接至 task_nav Action Server')
        else:
            rospy.logwarn(
                '[seu_nav] 等待 task_nav 超时（%ds），服务仍将启动，'
                '导航时会再次尝试连接', ACTION_CONNECT_TIMEOUT_SEC
            )

        # --------------------------------------------------
        # 3. 注册 ROS Service
        # --------------------------------------------------
        self.service = rospy.Service(SERVICE_NAME, seu_nav, self._handle_nav)
        rospy.loginfo('[seu_nav] 导航服务已就绪: %s', SERVICE_NAME)
        
        available_names = list(self.waypoints_data.get('waypoints', {}).keys())
        rospy.loginfo('[seu_nav] 已加载 %d 个航点: %s', len(available_names), available_names)

    # ========================================================
    # 航点加载
    # ========================================================
    def _load_waypoints(self, yaml_path):
        """完全对齐作者 waypoint_navigator.py 的读取方法。

        直接 safe_load 整个 YAML 文件，不改变其原始结构。这保证了
        以后直接替换 YAML 文件时 "到手即用"。

        Args:
            yaml_path (str): YAML 文件的绝对路径。

        Returns:
            dict: 原始的 YAML 字典数据。
        """
        waypoints_data = {'waypoints': {}}
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        waypoints_data = data
            except Exception as e:
                rospy.logerr('[seu_nav] 读取航点文件失败: %s', e)
        else:
            rospy.logwarn('[seu_nav] 航点文件不存在: %s', yaml_path)
            
        return waypoints_data

    # ========================================================
    # 服务回调
    # ========================================================
    def _handle_nav(self, req):
        """处理导航服务请求。

        流程：
            1. 将请求中的地点名称标准化（小写、下划线→空格）
            2. 在航点字典中查找对应坐标
            3. 构造 TaskNavGoal 并通过 Action Client 发送
            4. 等待导航结果并返回

        Args:
            req: seu_navRequest，包含 req.des (str) 目标地点名称。

        Returns:
            seu_navResponse: state (bool) 是否成功, errormsg (str) 错误信息。
        """
        response = seu_navResponse()

        # --- 1. 尝试查找航点 (直接提取 或 兼容匹配) ---
        req_location = req.des.strip()
        rospy.loginfo('[seu_nav] 收到导航请求: "%s"', req_location)
        
        waypoints_dict = self.waypoints_data.get('waypoints', {})
        
        # 首选：与 YAML 文件中键名完全一致
        wp = waypoints_dict.get(req_location)
        matched_name = req_location
        
        # 兼容性寻找：忽略大小写、以及下划线和空格的差别
        if not wp:
            normalized_req = req_location.lower().replace('_', ' ')
            for key, val in waypoints_dict.items():
                if key.lower().replace('_', ' ') == normalized_req:
                    wp = val
                    matched_name = key
                    break

        if not wp:
            err = '未知地点: "{}"，目前 YAML 文件中的可用地点有: {}'.format(
                req_location, list(waypoints_dict.keys())
            )
            rospy.logerr('[seu_nav] %s', err)
            response.state = False
            response.errormsg = err
            return response

        # --- 2. 构造导航目标 (按照 waypoint_navigator 的写法) ---
        goal = TaskNavGoal()
        
        # 1) 构造 nav_pose
        nav_p = PoseStamped()
        nav_p.header.frame_id = 'map'
        nav_p.header.stamp = rospy.Time.now()
        
        # 与作者脚本保持一致的高度对齐层级结构取值
        nav_p.pose.position.x = wp['nav_pose']['position']['x']
        nav_p.pose.position.y = wp['nav_pose']['position']['y']
        nav_p.pose.position.z = wp['nav_pose']['position']['z']
        nav_p.pose.orientation.x = wp['nav_pose']['orientation']['x']
        nav_p.pose.orientation.y = wp['nav_pose']['orientation']['y']
        nav_p.pose.orientation.z = wp['nav_pose']['orientation']['z']
        nav_p.pose.orientation.w = wp['nav_pose']['orientation']['w']
        
        # 2) 构造 pid_pose
        pid_p = PoseStamped()
        pid_p.header.frame_id = 'map'
        pid_p.header.stamp = rospy.Time.now()
        
        pid_p.pose.position.x = wp['pid_pose']['position']['x']
        pid_p.pose.position.y = wp['pid_pose']['position']['y']
        pid_p.pose.position.z = wp['pid_pose']['position']['z']
        pid_p.pose.orientation.x = wp['pid_pose']['orientation']['x']
        pid_p.pose.orientation.y = wp['pid_pose']['orientation']['y']
        pid_p.pose.orientation.z = wp['pid_pose']['orientation']['z']
        pid_p.pose.orientation.w = wp['pid_pose']['orientation']['w']

        goal.nav_pose = nav_p
        goal.pid_pose = pid_p

        rospy.loginfo('[seu_nav] 正在导航至 "%s" (实际匹配航点="%s", nav_x=%.2f, nav_y=%.2f)',
                      req_location, matched_name, nav_p.pose.position.x, nav_p.pose.position.y)

        # --- 4. 发送导航目标并等待结果 ---
        # 先确认 Action Server 可用
        if not self.nav_client.wait_for_server(rospy.Duration(ACTION_CONNECT_TIMEOUT_SEC)):
            err = 'task_nav Action Server 不可用'
            rospy.logerr('[seu_nav] %s', err)
            response.state = False
            response.errormsg = err
            return response

        self.nav_client.send_goal(goal)
        finished = self.nav_client.wait_for_result(rospy.Duration(self.timeout))

        if not finished:
            # 超时，取消目标
            self.nav_client.cancel_goal()
            err = '导航超时 ({}s)'.format(self.timeout)
            rospy.logwarn('[seu_nav] %s', err)
            response.state = False
            response.errormsg = err
            return response

        # 获取导航结果
        result = self.nav_client.get_result()
        if result and result.success:
            rospy.loginfo('[seu_nav] 导航成功: "%s"', matched_name)
            response.state = True
            response.errormsg = ''
        else:
            state_code = self.nav_client.get_state()
            err = '导航失败 (ActionState={})'.format(state_code)
            rospy.logwarn('[seu_nav] %s', err)
            response.state = False
            response.errormsg = err

        return response

    def spin(self):
        """阻塞运行，等待服务请求。"""
        rospy.spin()


# ============================================================
# 入口
# ============================================================
if __name__ == '__main__':
    try:
        node = NavServiceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass