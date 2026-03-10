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
    """方案 B 服务骨架。

    这里负责：

    - 校验 scope 是否存在
    - 维护黑名单区域
    - 暴露四类查询入口
    - 在感知未实现时给出统一的占位返回
    """

    def __init__(
        self,
        anchor_catalog: Optional[NavAnchorCatalog],
        blacklist_manager: Optional[BlacklistManager] = None,
    ):
        self.anchor_catalog = anchor_catalog
        self.blacklist_manager = blacklist_manager or BlacklistManager()

    def handle_find(self, target_class: str, search_scope: str, timeout_sec: float) -> QueryResult:
        """普通找物入口。"""

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
        """关系找物入口。"""

        # 当前阶段先记录关系请求的上下文。
        # 真正的关系推理会在感知层接入后落到 query_reasoner 中。
        result = self._handle_stubbed_query(
            query_mode="RELATE",
            target_class=target_class,
            reference_class=reference_class,
            search_scope=search_scope,
            timeout_sec=timeout_sec,
        )
        if result.status_code == "NOT_IMPLEMENTED":
            result.message = f"{result.message}，当前请求关系为 {relation}"
        return result

    def handle_count(self, target_class: str, search_scope: str, timeout_sec: float) -> QueryResult:
        """计数入口。"""

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
        """属性分析入口。"""

        result = self._handle_stubbed_query(
            query_mode="ANALYZE",
            target_class=target_class,
            reference_class="",
            search_scope=search_scope,
            timeout_sec=timeout_sec,
        )
        if result.status_code == "NOT_IMPLEMENTED":
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
        """处理当前阶段的统一占位逻辑。

        这里先把真正的边界条件收敛好：

        - scope 必须能解析到锚点
        - timeout 必须大于 0
        - 感知未实现时明确返回 NOT_IMPLEMENTED
        """

        if timeout_sec <= 0:
            return QueryResult(
                success=False,
                status_code="INVALID_QUERY",
                message="timeout_sec 必须大于 0。",
                query_mode=query_mode,
                target_class=target_class,
                reference_class=reference_class,
            )

        if self.anchor_catalog is not None and not self.anchor_catalog.anchors_for(search_scope):
            return QueryResult(
                success=False,
                status_code="UNKNOWN_SCOPE",
                message=f"搜索范围 {search_scope} 没有可用锚点。",
                query_mode=query_mode,
                target_class=target_class,
                reference_class=reference_class,
            )

        return QueryResult(
            success=False,
            status_code="NOT_IMPLEMENTED",
            message="感知侧和抵近复核逻辑尚未实现，当前仅完成接口与控制骨架。",
            query_mode=query_mode,
            target_class=target_class,
            reference_class=reference_class,
        )
