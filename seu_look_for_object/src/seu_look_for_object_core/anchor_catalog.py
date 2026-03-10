"""导航锚点目录解析器。

当前设计约定由 `seu_nav` 提供锚点 YAML。
本模块只做一件事：把类似 `kitchen_0`、`kitchen_1` 这种命名的锚点
归并到同一个 scope 下，供找物服务按 scope 巡航。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Any
import re
import yaml


ANCHOR_NAME_PATTERN = re.compile(r"^(?P<scope>.+)_(?P<index>\d+)$")


@dataclass(frozen=True)
class AnchorPoint:
    """单个导航锚点。

    Attributes:
        name: 锚点原始名称，例如 `kitchen_0`
        scope: 归属区域名称，例如 `kitchen`
        index: 该区域下的顺序编号
        x: 地图坐标 x
        y: 地图坐标 y
        yaw: 朝向角，单位为弧度
    """

    name: str
    scope: str
    index: int
    x: float
    y: float
    yaw: float


class NavAnchorCatalog:
    """按 scope 管理导航锚点。

    这个类只关心锚点文件的结构化读取，不负责发布导航目标。
    """

    def __init__(self, anchors_by_scope: Mapping[str, Iterable[AnchorPoint]]):
        self._anchors_by_scope: Dict[str, List[AnchorPoint]] = {
            scope: sorted(list(anchors), key=lambda anchor: anchor.index)
            for scope, anchors in anchors_by_scope.items()
        }

    @classmethod
    def from_yaml_file(cls, yaml_path: str | Path) -> "NavAnchorCatalog":
        """从 YAML 文件中读取锚点。

        YAML 的顶层结构允许是：

        - `{anchor_name: {x: ..., y: ..., yaw: ...}}`
        - 或包含 `points`/`anchors` 这样的嵌套字典
        """

        raw_mapping = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8")) or {}

        if "points" in raw_mapping and isinstance(raw_mapping["points"], dict):
            raw_mapping = raw_mapping["points"]
        elif "anchors" in raw_mapping and isinstance(raw_mapping["anchors"], dict):
            raw_mapping = raw_mapping["anchors"]

        return cls.from_mapping(raw_mapping)

    @classmethod
    def from_mapping(cls, raw_points: Mapping[str, Mapping[str, Any]]) -> "NavAnchorCatalog":
        """从内存字典构建锚点目录。"""

        anchors_by_scope: Dict[str, List[AnchorPoint]] = {}

        for name, payload in raw_points.items():
            match = ANCHOR_NAME_PATTERN.match(name)
            if match is None:
                # 不符合 `<scope>_<index>` 命名约定的点位直接忽略，
                # 避免把导航系统里的辅助点误当作巡航锚点。
                continue

            scope = match.group("scope")
            index = int(match.group("index"))
            anchor = AnchorPoint(
                name=name,
                scope=scope,
                index=index,
                x=float(payload.get("x", 0.0)),
                y=float(payload.get("y", 0.0)),
                yaw=float(payload.get("yaw", 0.0)),
            )
            anchors_by_scope.setdefault(scope, []).append(anchor)

        return cls(anchors_by_scope)

    def scopes(self) -> List[str]:
        """返回当前已知的所有搜索 scope。"""

        return sorted(self._anchors_by_scope.keys())

    def anchors_for(self, scope: str) -> List[AnchorPoint]:
        """返回某个 scope 下按编号排序的所有锚点。"""

        return list(self._anchors_by_scope.get(scope, []))
