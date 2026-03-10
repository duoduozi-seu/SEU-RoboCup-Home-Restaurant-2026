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
    - ✅ 从 YAML 中读取并分组精细巡航锚点名称（如 Kitchen_0, Kitchen_1）
    - ✅ 按 index 序号排序，提供有序的巡航顺序
    - ❌ 不负责坐标解析（交给 seu_nav）
    - ❌ 不负责发布导航目标（交给 SMACH 或 seu_nav）
    """

    def __init__(self, anchors_by_scope: Mapping[str, Iterable[AnchorPoint]]):
        # 初始化内部字典。
        # 核心逻辑：对每一个区域（scope）下的锚点列表进行排序
        # 排序依据是我们在 AnchorPoint 中解析出来的 index（即名称后缀的数字）
        self._anchors_by_scope: Dict[str, List[AnchorPoint]] = {
            scope: sorted(list(anchors), key=lambda anchor: anchor.index)
            for scope, anchors in anchors_by_scope.items()
        }

        # 对每个区域做简单的完整性与逻辑检查
        for scope, anchors in self._anchors_by_scope.items():
            # 分支：如果该区域没有任何有效锚点
            if not anchors:
                logger.warning(
                    "[AnchorCatalog] scope '%s' 没有任何有效的精细锚点，"
                    "该区域将直接进入感知环节（即原地搜索，不巡航）", scope
                )
                continue

            # 分支：检查第一个锚点是否从 0 开始
            # 在我们的约定中，巡航点应该是 Kitchen_0, Kitchen_1... 
            # 如果从 Kitchen_5 开始，虽然不影响运行，但可能意味着 YAML 配置有误。
            first_index = anchors[0].index
            if first_index != 0:
                logger.warning(
                    "[AnchorCatalog] scope '%s' 的精细锚点不是从 _0 开始的，"
                    "实际是从 _%d（%s）开始，将按现有序号顺序巡航",
                    scope, first_index, anchors[0].name
                )

    @classmethod
    def from_yaml_file(cls, yaml_path: str | Path) -> "NavAnchorCatalog":
        """【关键函数】从指定的 waypoints.yaml 文件中读取并构建锚点目录。
        
        这个函数会自动处理不同格式的 YAML 结构，确保兼容性。
        """

        # 1. 读取文件内容并使用 yaml 库解析成 Python 字典
        #    Path(yaml_path).read_text(...) 直接读取全文
        raw_data = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8")) or {}

        # 2. 这里的 if-elif 分支是为了兼容各种版本的 waypoints.yaml 格式。
        #    有的版本把数据放在 'waypoints' 键下，有的直接放在顶层。
        if "waypoints" in raw_data and isinstance(raw_data["waypoints"], dict):
            raw_mapping = raw_data["waypoints"]
        elif "points" in raw_data and isinstance(raw_data["points"], dict):
            raw_mapping = raw_data["points"]
        elif "anchors" in raw_data and isinstance(raw_data["anchors"], dict):
            raw_mapping = raw_data["anchors"]
        else:
            # 兜底分支：如果以上键都没找到，假设整个文件顶层就是大字典
            raw_mapping = raw_data

        # 3. 提取字典的所有键（即所有地点的名称列表），传给下一个处理函数
        return cls.from_name_list(list(raw_mapping.keys()))

    @classmethod
    def from_name_list(cls, names: List[str]) -> "NavAnchorCatalog":
        """工具函数：从原始地名列表中筛选出符合“精细巡航点”要求的名字。
        
        它会过滤掉像 'Kitchen' 这种模糊地点，只留下 'Kitchen_0' 这种具体地点。
        """

        anchors_by_scope: Dict[str, List[AnchorPoint]] = {}

        # 循环遍历每一个名字（比如：'Kitchen', 'Kitchen_0', 'LivingRoom_1'）
        for name in names:
            # 使用前面定义的正则表达式进行匹配
            match = ANCHOR_NAME_PATTERN.match(name)
            
            # 分支：如果不符合 <名称>_<数字> 的格式
            if match is None:
                # 说明这是“宽泛锚点”或“起始点”，在找物巡航中不需要它，直接跳过。
                continue

            # 如果匹配成功，从正则组里提取出区域名(scope)和序号(index)
            scope = match.group("scope")
            index = int(match.group("index"))
            
            # 封装成对象
            anchor = AnchorPoint(name=name, scope=scope, index=index)
            
            # 按照 scope 进行归类。
            # setdefault 的意思是：如果该 scope 还没在字典里，就先初始化一个空列表 []。
            anchors_by_scope.setdefault(scope, []).append(anchor)

        # 4. 最后实例化类对象。此时会触发 __init__ 里的排序逻辑。
        catalog = cls(anchors_by_scope)

        # 检查逻辑：如果解析完一个精细锚点都没找到，打印警告。
        if not anchors_by_scope:
            logger.warning(
                "[AnchorCatalog] YAML 中未找到任何符合 <区域>_<数字> 格式的精细巡航锚点。"
                "这会导致机器人直接在原地开始搜索，而不会移动去房间各处看。"
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
