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
    """ROS 节点包装器。"""

    def __init__(self):
        anchor_yaml_path = rospy.get_param("~anchor_yaml_path", "")
        self.query_service = ObjectQueryService(anchor_catalog=self._load_anchor_catalog(anchor_yaml_path))

        # 统一在一个节点里注册四类服务，便于共享锚点目录、黑名单和后续感知实例。
        self.find_service = rospy.Service("find_object_service", FindObject, self.handle_find)
        self.relate_service = rospy.Service("relate_object_service", RelateObject, self.handle_relate)
        self.count_service = rospy.Service("count_object_service", CountObject, self.handle_count)
        self.analyze_service = rospy.Service("analyze_object_service", AnalyzeObject, self.handle_analyze)

    def _load_anchor_catalog(self, anchor_yaml_path: str):
        """按参数读取导航锚点。

        参数为空时不报错，允许开发者先把服务节点拉起来，只做接口联调。
        """

        if not anchor_yaml_path:
            rospy.logwarn("未提供 ~anchor_yaml_path，当前仅能做接口联调。")
            return None

        yaml_path = Path(anchor_yaml_path)
        if not yaml_path.exists():
            rospy.logwarn("锚点文件不存在：%s", yaml_path)
            return None

        rospy.loginfo("加载导航锚点文件：%s", yaml_path)
        return NavAnchorCatalog.from_yaml_file(yaml_path)

    def handle_find(self, request: FindObject.Request) -> FindObjectResponse:
        """普通找物服务回调。"""

        result = self.query_service.handle_find(
            target_class=request.target_class,
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        return FindObjectResponse(result=self._to_ros_result(result))

    def handle_relate(self, request: RelateObject.Request) -> RelateObjectResponse:
        """关系找物服务回调。"""

        result = self.query_service.handle_relate(
            target_class=request.target_class,
            reference_class=request.reference_class,
            relation=request.relation,
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        return RelateObjectResponse(result=self._to_ros_result(result))

    def handle_count(self, request: CountObject.Request) -> CountObjectResponse:
        """计数服务回调。"""

        result = self.query_service.handle_count(
            target_class=request.target_class,
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        return CountObjectResponse(result=self._to_ros_result(result))

    def handle_analyze(self, request: AnalyzeObject.Request) -> AnalyzeObjectResponse:
        """属性分析服务回调。"""

        result = self.query_service.handle_analyze(
            target_class=request.target_class,
            metric=request.metric,
            search_scope=request.search_scope,
            timeout_sec=request.timeout_sec,
        )
        return AnalyzeObjectResponse(result=self._to_ros_result(result))

    def _to_ros_result(self, result) -> ObjectQueryResult:
        """把纯 Python 结果转换成 ROS 消息。"""

        ros_result = ObjectQueryResult()
        ros_result.success = result.success
        ros_result.status_code = result.status_code
        ros_result.message = result.message
        ros_result.query_mode = result.query_mode
        ros_result.target_class = result.target_class
        ros_result.reference_class = result.reference_class
        ros_result.resolved_label = result.resolved_label
        ros_result.count = result.count
        ros_result.selected_object = self._to_ros_detection(result.selected_object)
        ros_result.candidate_objects = [self._to_ros_detection(item) for item in result.candidate_objects]
        return ros_result

    def _to_ros_detection(self, detection) -> ObjectDetectionResult:
        """把纯 Python 候选结构转换成 ROS 消息。"""

        ros_detection = ObjectDetectionResult()
        ros_detection.label = detection.label
        ros_detection.score = detection.score
        ros_detection.pose = PoseStamped()
        ros_detection.pose_valid = detection.pose_valid
        ros_detection.bbox = list(detection.bbox)
        ros_detection.distance = detection.distance
        return ros_detection


def main():
    """节点主入口。"""

    rospy.init_node("object_query_server")
    ObjectQueryRosNode()
    rospy.loginfo("方案 B 找物多服务节点已启动，当前为接口骨架模式。")
    rospy.spin()


if __name__ == "__main__":
    main()
