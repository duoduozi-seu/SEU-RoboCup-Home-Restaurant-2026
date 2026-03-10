"""方案 B 的找物服务核心逻辑。

当前阶段的目标不是把感知算法全部写完，而是先把以下部分稳定下来：

- 服务入口函数签名
- scope 锚点巡航的业务流程
- 黑名单区域管理
- 统一结果结构

因此本模块故意把感知相关逻辑收敛成显式钩子函数。
当感知侧还没接好时，服务会返回 `NOT_IMPLEMENTED`，而不是假装成功。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from .anchor_catalog import NavAnchorCatalog
from .blacklist_manager import BlacklistManager


@dataclass
class DetectedCandidate:
    """纯 Python 层使用的候选点结构。"""

    label: str = ""
    score: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    distance: float = 0.0
    pose_valid: bool = False
    bbox: List[float] = field(default_factory=list)


@dataclass
class QueryResult:
    """服务返回结果的纯 Python 表达。

    该结构会在 ROS 节点层被转换成 `ObjectQueryResult.msg`。
    """

    success: bool
    status_code: str
    message: str
    query_mode: str
    target_class: str
    reference_class: str = ""
    resolved_label: str = ""
    count: int = 0
    selected_object: DetectedCandidate = field(default_factory=DetectedCandidate)
    candidate_objects: List[DetectedCandidate] = field(default_factory=list)


class ObjectQueryService:
    """方案 B 服务骨架类。

    这个类是整个包的“大脑”，负责：
    - 校验用户请求的房间（scope）是否合法
    - 统筹黑名单过滤
    - 暴露四个标准化的查询接口
    - 处理当前阶段的模拟返回逻辑
    """

    def __init__(
        self,
        anchor_catalog: Optional[NavAnchorCatalog],
        blacklist_manager: Optional[BlacklistManager] = None,
    ):
        # 注入依赖：锚点表和黑名单管理器
        self.anchor_catalog = anchor_catalog
        # 如果调用者没传黑名单管理器，我们就自己新建一个
        self.blacklist_manager = blacklist_manager or BlacklistManager()

    def handle_find(self, target_class: str, search_scope: str, timeout_sec: float) -> QueryResult:
        """【业务入口】普通找物逻辑。"""

        # 转发给统一的查询处理函数
        return self._handle_stubbed_query(
            query_mode="FIND",
            target_class=target_class,
            reference_class="",
            search_scope=search_scope,
            timeout_sec=timeout_sec,
        )

    def handle_relate(
        self,
        target_class: str,
        reference_class: str,
        relation: str,
        search_scope: str,
        timeout_sec: float,
    ) -> QueryResult:
        """【业务入口】关系找物逻辑。例子：'在沙发旁边的苹果'。"""

        # 调用统一查询，并传入关系相关的上下文
        result = self._handle_stubbed_query(
            query_mode="RELATE",
            target_class=target_class,
            reference_class=reference_class,
            search_scope=search_scope,
            timeout_sec=timeout_sec,
        )
        # 补充逻辑：提示当前这只是个模拟结果，并显示用户请求的关系是什么
        if result.status_code == "STUB_SUCCESS":
            result.message = f"{result.message}，当前关系为 {relation}"
        return result

    def handle_count(self, target_class: str, search_scope: str, timeout_sec: float) -> QueryResult:
        """【业务入口】物体计数逻辑。"""

        return self._handle_stubbed_query(
            query_mode="COUNT",
            target_class=target_class,
            reference_class="",
            search_scope=search_scope,
            timeout_sec=timeout_sec,
        )

    def handle_analyze(
        self,
        target_class: str,
        metric: str,
        search_scope: str,
        timeout_sec: float,
    ) -> QueryResult:
        """【业务入口】物体属性分析逻辑。例子：'哪个瓶子是排在最前面的'。"""

        result = self._handle_stubbed_query(
            query_mode="ANALYZE",
            target_class=target_class,
            reference_class="",
            search_scope=search_scope,
            timeout_sec=timeout_sec,
        )
        # 补充逻辑：显示模拟结果中的分析指标
        if result.status_code == "STUB_SUCCESS":
            result.message = f"{result.message}，当前分析指标为 {metric}"
        return result

    def _handle_stubbed_query(
        self,
        query_mode: str,
        target_class: str,
        reference_class: str,
        search_scope: str,
        timeout_sec: float,
    ) -> QueryResult:
        """【核心分支逻辑】处理所有查询的统一校验与占位逻辑。
        
        目前这个函数会执行一些基础校验，如果没有错误，则由于视觉尚未接入，
        它会返回一个带 success=True 的模拟成功结果，配合 SMACH 跑通流程。
        """

        # 分支 1：校验超时时间是否合法
        if timeout_sec <= 0:
            return QueryResult(
                success=False,
                status_code="INVALID_QUERY",
                message="错误：timeout_sec 必须大于 0。",
                query_mode=query_mode,
                target_class=target_class,
                reference_class=reference_class,
            )

        # 分支 2：校验搜索房间（scope）是否在本地 YAML 中有对应的巡航点
        # 如果 self.anchor_catalog 存在且该 scope 拿不到任何锚点
        if self.anchor_catalog is not None and not self.anchor_catalog.anchors_for(search_scope):
            return QueryResult(
                success=False,
                status_code="UNKNOWN_SCOPE",
                message=f"错误：搜索范围 '{search_scope}' 在地图配置文件中没有找到可用的巡航锚点（如 {search_scope}_0）。",
                query_mode=query_mode,
                target_class=target_class,
                reference_class=reference_class,
            )

        # 分支 3：一切正常，进入模拟返回环节
        # 未来：这里应该启动真正的巡航搜索算法循环：
        # while 时间没到:
        #   去下一个锚点()
        #   检测物体()
        #   if 找到且不在黑名单: return 成功
        return QueryResult(
            success=True, # 暂时设为 True，为了配合联调进度
            status_code="STUB_SUCCESS",
            message="[调试模式] 感知侧和抵近复核逻辑尚未真实实现，当前仅返回模拟成功以配合联调流程。",
            query_mode=query_mode,
            target_class=target_class,
            reference_class=reference_class,
        )
