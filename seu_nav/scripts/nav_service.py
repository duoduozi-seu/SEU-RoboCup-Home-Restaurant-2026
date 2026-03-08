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
        # 使用常量中的硬编码路径加载配置
        self.waypoints = self._load_waypoints(WAYPOINTS_YAML_PATH)

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
        rospy.loginfo('[seu_nav] 已加载 %d 个航点: %s',
                      len(self.waypoints), list(self.waypoints.keys()))

    # ========================================================
    # 航点加载
    # ========================================================
    def _load_waypoints(self, yaml_path):
        """从 YAML 文件加载航点字典。

        YAML 格式示例（适配 tracer_nav 双位姿）：
            waypoints:
              living_room:
                nav_pose:
                  position:  {x: 1.0, y: 2.0, z: 0.0}
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                pid_pose:
                  position:  {x: 1.0, y: 2.0, z: 0.0}
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

        Args:
            yaml_path (str): YAML 文件的绝对路径。

        Returns:
            dict: 键为航点名称（全小写，空格分隔），值为坐标字典。
        """
        if not os.path.exists(yaml_path):
            rospy.logwarn('[seu_nav] 航点文件不存在: %s，将使用空航点表', yaml_path)
            return {}

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr('[seu_nav] 读取航点文件失败: %s', e)
            return {}

        if not data or 'waypoints' not in data:
            rospy.logwarn('[seu_nav] 航点文件格式错误，缺少 "waypoints" 键')
            return {}

        # 将航点名称统一转为小写、下划线替换为空格，方便匹配
        raw = data['waypoints']
        waypoints = {}
        for name, coords in raw.items():
            normalized = name.lower().replace('_', ' ')
            waypoints[normalized] = coords
            # 同时保留原始名称（小写）作为别名，确保两种写法都能匹配
            lower_name = name.lower()
            if lower_name != normalized:
                waypoints[lower_name] = coords
        return waypoints

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

        # --- 1. 地点名称标准化 ---
        location = req.des.lower().replace('_', ' ').strip()
        rospy.loginfo('[seu_nav] 收到导航请求: "%s"', location)

        # --- 2. 查找航点 ---
        if location not in self.waypoints:
            err = '未知地点: "{}"，可用地点: {}'.format(
                location, list(self.waypoints.keys())
            )
            rospy.logerr('[seu_nav] %s', err)
            response.state = False
            response.errormsg = err
            return response

        wp = self.waypoints[location]

        # --- 3. 构造导航目标 ---
        goal = TaskNavGoal()
        
        # 1) 构造 nav_pose
        nav_p = PoseStamped()
        nav_p.header.frame_id = 'map'
        nav_p.header.stamp = rospy.Time.now()
        
        nwp = wp.get('nav_pose', wp)  # 回退到直接取wp，兼容旧有格式
        npos = nwp.get('position', {'x':0, 'y':0, 'z':0})
        nori = nwp.get('orientation', {'x':0, 'y':0, 'z':0, 'w':1})
        
        nav_p.pose.position.x = npos.get('x', 0.0)
        nav_p.pose.position.y = npos.get('y', 0.0)
        nav_p.pose.position.z = npos.get('z', 0.0)
        nav_p.pose.orientation.x = nori.get('x', 0.0)
        nav_p.pose.orientation.y = nori.get('y', 0.0)
        nav_p.pose.orientation.z = nori.get('z', 0.0)
        nav_p.pose.orientation.w = nori.get('w', 1.0)
        
        # 2) 构造 pid_pose
        pid_p = PoseStamped()
        pid_p.header.frame_id = 'map'
        pid_p.header.stamp = rospy.Time.now()
        
        pwp = wp.get('pid_pose', wp)  # 回退到直接取wp，兼容旧有格式
        ppos = pwp.get('position', {'x':0, 'y':0, 'z':0})
        pori = pwp.get('orientation', {'x':0, 'y':0, 'z':0, 'w':1})
        
        pid_p.pose.position.x = ppos.get('x', 0.0)
        pid_p.pose.position.y = ppos.get('y', 0.0)
        pid_p.pose.position.z = ppos.get('z', 0.0)
        pid_p.pose.orientation.x = pori.get('x', 0.0)
        pid_p.pose.orientation.y = pori.get('y', 0.0)
        pid_p.pose.orientation.z = pori.get('z', 0.0)
        pid_p.pose.orientation.w = pori.get('w', 1.0)

        goal.nav_pose = nav_p
        goal.pid_pose = pid_p

        rospy.loginfo('[seu_nav] 正在导航至 "%s" (nav_x=%.2f, nav_y=%.2f)',
                      location, nav_p.pose.position.x, nav_p.pose.position.y)

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
            rospy.loginfo('[seu_nav] 导航成功: "%s"', location)
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