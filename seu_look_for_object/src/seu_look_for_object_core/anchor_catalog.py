"""导航锚点目录解析器。

设计说明：
    本模块从 seu_nav 共用的 waypoints.yaml 中读取锚点**名称**。
    实际导航由 seu_nav 的 nav_service 执行，本模块不关心坐标。

    YAML 中的航点命名约定：
    - 宽泛锚点：`Kitchen`、`LivingRoom` —— 每个房间一个，用于普通 go-to 任务
    - 精细巡航锚点：`Kitchen_0`、`Kitchen_1` —— 每个房间至少一个，
      用于 look_for_object 逐点巡航搜索

    本模块只提取 `<Scope>_<数字>` 格式的锚点名，按 scope 分组并排序，
    供找物服务决定巡航顺序。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Any, Optional
import re
import yaml


# 匹配 "Kitchen_0"、"living_room_1" 等命名的正则
# scope 部分允许包含下划线（贪婪匹配），最后一个 _数字 作为 index
ANCHOR_NAME_PATTERN = re.compile(r"^(?P<scope>.+)_(?P<index>\d+)$")

# 模块级 logger，在 ROS 环境中会被 rospy 接管
logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class AnchorPoint:
    """单个导航锚点（仅存储名称信息，不含坐标）。

    Attributes:
        name: 锚点原始名称，例如 `Kitchen_0`，与 waypoints.yaml 中的键名一致，
              可直接作为 seu_nav 服务的目标地点名
        scope: 归属区域名称，例如 `Kitchen`
        index: 该区域下的顺序编号
    """

    name: str
    scope: str
    index: int


class NavAnchorCatalog:
    """按 scope 管理导航锚点名称。

    职责边界：
    - ✅ 从 YAML 中读取并分组精细巡航锚点名称
    - ✅ 按 index 排序，提供有序的巡航顺序
    - ❌ 不负责坐标解析（交给 seu_nav）
    - ❌ 不负责发布导航目标（交给 seu_nav）
    """

    def __init__(self, anchors_by_scope: Mapping[str, Iterable[AnchorPoint]]):
        self._anchors_by_scope: Dict[str, List[AnchorPoint]] = {
            scope: sorted(list(anchors), key=lambda anchor: anchor.index)
            for scope, anchors in anchors_by_scope.items()
        }

        # 对每个 scope 做简单的完整性检查
        for scope, anchors in self._anchors_by_scope.items():
            if not anchors:
                logger.warning(
                    "[AnchorCatalog] scope '%s' 没有任何有效的精细锚点，"
                    "该区域将直接进入感知环节（不巡航）", scope
                )
                continue

            first_index = anchors[0].index
            if first_index != 0:
                logger.warning(
                    "[AnchorCatalog] scope '%s' 的精细锚点不是从 _0 开始的，"
                    "最小编号为 _%d（%s），将按现有顺序巡航",
                    scope, first_index, anchors[0].name
                )

    @classmethod
    def from_yaml_file(cls, yaml_path: str | Path) -> "NavAnchorCatalog":
        """从 waypoints.yaml 文件中读取精细巡航锚点名称。

        兼容以下 YAML 顶层结构（按优先级）：
        1. `waypoints: {name: ...}`  —— seu_nav 使用的标准格式
        2. `points: {name: ...}` 或 `anchors: {name: ...}`
        3. 顶层直接是 `{name: ...}`
        """

        raw_data = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8")) or {}

        # 按优先级尝试提取航点字典
        if "waypoints" in raw_data and isinstance(raw_data["waypoints"], dict):
            raw_mapping = raw_data["waypoints"]
        elif "points" in raw_data and isinstance(raw_data["points"], dict):
            raw_mapping = raw_data["points"]
        elif "anchors" in raw_data and isinstance(raw_data["anchors"], dict):
            raw_mapping = raw_data["anchors"]
        else:
            # 顶层本身就是航点字典
            raw_mapping = raw_data

        return cls.from_name_list(list(raw_mapping.keys()))

    @classmethod
    def from_name_list(cls, names: List[str]) -> "NavAnchorCatalog":
        """从航点名称列表构建锚点目录。

        只提取符合 `<scope>_<数字>` 命名约定的条目，
        其余名称（如 `Kitchen`、`StartPoint`）视为宽泛锚点，自动忽略。
        """

        anchors_by_scope: Dict[str, List[AnchorPoint]] = {}

        for name in names:
            match = ANCHOR_NAME_PATTERN.match(name)
            if match is None:
                # 不符合 `<scope>_<index>` 命名约定 → 宽泛锚点，跳过
                continue

            scope = match.group("scope")
            index = int(match.group("index"))
            anchor = AnchorPoint(name=name, scope=scope, index=index)
            anchors_by_scope.setdefault(scope, []).append(anchor)

        catalog = cls(anchors_by_scope)

        if not anchors_by_scope:
            logger.warning(
                "[AnchorCatalog] YAML 中未找到任何精细巡航锚点"
                "（需要 <Scope>_<数字> 格式的命名），"
                "所有搜索请求将直接进入感知环节"
            )

        return catalog

    def scopes(self) -> List[str]:
        """返回当前已知的所有搜索 scope（区域名称列表）。"""

        return sorted(self._anchors_by_scope.keys())

    def anchors_for(self, scope: str) -> List[AnchorPoint]:
        """返回某个 scope 下按编号排序的所有精细巡航锚点。

        返回的锚点名称可直接作为 seu_nav 服务的目标地点名使用。
        如果该 scope 没有精细锚点，返回空列表。
        """

        return list(self._anchors_by_scope.get(scope, []))

    def anchor_names_for(self, scope: str) -> List[str]:
        """便捷方法：直接返回某个 scope 下的锚点名称列表。

        返回值可直接传给 seu_nav 服务的 req.des 字段。
        """

        return [anchor.name for anchor in self.anchors_for(scope)]
