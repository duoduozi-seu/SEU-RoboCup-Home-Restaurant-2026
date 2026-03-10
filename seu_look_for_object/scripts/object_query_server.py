#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案 B 下的多服务找物节点。

本节点当前只做三件事：

1. 读取导航锚点文件，建立 scope 到锚点列表的映射
2. 注册四个语义清晰的 ROS Service
3. 把纯 Python 结果结构转换为 ROS 消息返回

注意：
当前版本刻意不实现感知算法本身，只把接口、业务边界和控制骨架稳定下来。
这样后续补 detector / projector / query_reasoner 时，不会再反复修改服务契约。
"""

from __future__ import annotations

from pathlib import Path
import rospy

from geometry_msgs.msg import PoseStamped

from seu_look_for_object.msg import ObjectDetectionResult, ObjectQueryResult
from seu_look_for_object.srv import (
    AnalyzeObject,
    AnalyzeObjectResponse,
    CountObject,
    CountObjectResponse,
    FindObject,
    FindObjectResponse,
    RelateObject,
    RelateObjectResponse,
)
from seu_look_for_object_core import NavAnchorCatalog, ObjectQueryService


class ObjectQueryRosNode:
    """ROS 节点包装器：负责将核心逻辑与 ROS Service 通信机制连接起来。"""

    def __init__(self):
        # 1. 获取寻路锚点的 YAML 文件路径参数
        #    第二个参数是默认值，指向 seu_nav 的标准配置文件
        anchor_yaml_path = rospy.get_param("~anchor_yaml_path", "/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_nav/config/waypoints.yaml")
        
        # 2. 调用私有方法加载锚点目录，并将结果传给核心推理服务 ObjectQueryService
        #    通过这种方式，ObjectQueryService 就不需要关心 ROS 环境，只看加载好的对象。
        self.query_service = ObjectQueryService(anchor_catalog=self._load_anchor_catalog(anchor_yaml_path))

        # 3. 统一在一个节点里注册四类服务。
        #    这样做的好处是：它们可以共享同一个感知模型实例、同一个导航锚点表、以及同一个误识别黑名单。
        self.find_service = rospy.Service("find_object_service", FindObject, self.handle_find)
        self.relate_service = rospy.Service("relate_object_service", RelateObject, self.handle_relate)
        self.count_service = rospy.Service("count_object_service", CountObject, self.handle_count)
        self.analyze_service = rospy.Service("analyze_object_service", AnalyzeObject, self.handle_analyze)

    def _load_anchor_catalog(self, anchor_yaml_path: str):
        """按参数读取导航锚点文件。
        
        这个方法负责从磁盘加载文件并处理可能的路径错误或文件缺失情况。
        """

        # 第一步：检查路径字符串是否为空
        # 如果开发者漏填了参数，程序会打印警告但不会闪退，允许在这个模式下只做接口调试。
        if not anchor_yaml_path:
            rospy.logwarn("未提供 ~anchor_yaml_path，当前仅能做接口联调（无法自动巡航）。")
            return None

        # 第二步：使用 pathlib 库检查物理文件是否存在
        yaml_path = Path(anchor_yaml_path)
        if not yaml_path.exists():
            rospy.logwarn("锚点文件不存在，请检查路径是否正确：%s", yaml_path)
            return None

        # 第三步：调用核心层的 NavAnchorCatalog 从 YAML 数据中解析出带有 _0 这种序号的巡航点
        rospy.loginfo("加载导航锚点文件：%s", yaml_path)
        return NavAnchorCatalog.from_yaml_file(yaml_path)

    def handle_find(self, request: FindObject.Request) -> FindObjectResponse:
        """普通找物服务的回调函数。
        
        当 SMACH 或命令行发起寻找请求时，ROS 会调用这个函数。
        """
        # A. 打印调试日志，方便在终端看到是谁在找什么
        rospy.loginfo("收到找物请求: target_class='%s', search_scope='%s'", 
                      request.target_class, request.search_scope)

        # B. 将请求转发给核心逻辑层处理
        #    注意这里解构了 request 里的字段：目标类别、搜索范围（房间名）、超时时间
        result = self.query_service.handle_find(
            target_class=request.target_class,
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        
        # C. 核心逻辑返回的是普通的 Python 对象（QueryResult），
        #    我们需要通过 _to_ros_result 将其转换为 ObjectQueryResult.msg 定义的 ROS 格式。
        return FindObjectResponse(result=self._to_ros_result(result))

    def handle_relate(self, request: RelateObject.Request) -> RelateObjectResponse:
        """关系找物服务回调：例如 '在沙发旁边的苹果'。"""

        # 同样转发给核心层，增加了参考物（reference_class）和空间关系描述（relation）
        result = self.query_service.handle_relate(
            target_class=request.target_class,
            reference_class=request.reference_class,
            relation=request.relation,
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        return RelateObjectResponse(result=self._to_ros_result(result))

    def handle_count(self, request: CountObject.Request) -> CountObjectResponse:
        """计数服务回调：例如 '厨房里有多少个瓶子'。"""

        result = self.query_service.handle_count(
            target_class=request.target_class,
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        return CountObjectResponse(result=self._to_ros_result(result))

    def handle_analyze(self, request: AnalyzeObject.Request) -> AnalyzeObjectResponse:
        """属性分析服务回调：例如 '哪个杯子是最大的'。"""

        result = self.query_service.handle_analyze(
            target_class=request.target_class,
            metric=request.metric,  # 分析指标，如 'biggest'
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        return AnalyzeObjectResponse(result=self._to_ros_result(result))

    def _to_ros_result(self, result) -> ObjectQueryResult:
        """工具方法：把纯 Python 的 QueryResult 结构体“翻译”成 ROS 自定义的消息类型。"""

        # 1. 实例化一个空的 ROS Message 对象
        ros_result = ObjectQueryResult()
        
        # 2. 依次赋值基础字段
        ros_result.success = result.success         # 是否查询成功
        ros_result.status_code = result.status_code # 状态码
        ros_result.message = result.message         # 补充提示文字
        ros_result.query_mode = result.query_mode   # 查询模式 (FIND/RELATE等)
        ros_result.target_class = result.target_class
        ros_result.reference_class = result.reference_class
        ros_result.resolved_label = result.resolved_label # 最终解析出来的具体标签名
        ros_result.count = result.count             # 物体数量
        
        # 3. 处理查找到的具体物体详情（嵌套消息）
        #    调用私有转换方法将候选点信息也转换成 ROS 格式
        ros_result.selected_object = self._to_ros_detection(result.selected_object)
        
        # 4. 如果有多个候选物体，使用列表推导式批量转换
        ros_result.candidate_objects = [self._to_ros_detection(item) for item in result.candidate_objects]
        
        return ros_result

    def _to_ros_detection(self, detection) -> ObjectDetectionResult:
        """工具方法：把具体的检测点信息（坐标、置信度等）翻译成 ROS 消息。"""

        ros_detection = ObjectDetectionResult()
        ros_detection.label = detection.label
        ros_detection.score = detection.score
        
        # 这里预留了一个标准的 PoseStamped 位姿，方便后续做三维投影时使用
        ros_detection.pose = PoseStamped()
        # 标记当前坐标是否有效（如果只是二维检测，则 pose_valid 为 False）
        ros_detection.pose_valid = detection.pose_valid
        
        # 转换坐标列表和距离
        ros_detection.bbox = list(detection.bbox)
        ros_detection.distance = detection.distance # 距离机器人的直线距离
        
        return ros_detection


def main():
    """节点的主启动函数。"""

    # 1. 初始化 ROS 节点，这就是你在命令行用 rosnode list 能看到的名称
    rospy.init_node("object_query_server")
    
    # 2. 实例化包装类。这个操作会触发 __init__，完成服务注册。
    ObjectQueryRosNode()
    
    # 3. 打印启动成功日志
    rospy.loginfo("方案 B 找物多服务节点已启动，当前为接口骨架模式。")
    
    # 4. 进入循环等待。没有这个函数的话，Python 脚本运行完 main 就会直接退出了。
    #    spin() 会让节点持续运行并监听 Service 请求。
    rospy.spin()


if __name__ == "__main__":
    main()
