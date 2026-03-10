"""误识别黑名单区域管理器。

当机器人抵近复核后发现候选点是误识别时，需要把该区域临时拉黑，
防止在同一个房间巡航时反复走向同一个错误候选点。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import List


@dataclass(frozen=True)
class BlacklistZone:
    """平面黑名单圆形区域。"""

    x: float
    y: float
    radius: float


class BlacklistManager:
    """管理候选点黑名单。
    
    当机器人走到某个物体面前复核，发现是误识别（比如把椅子腿看成了笔）时，
    就需要把那个位置拉黑，防止后续巡航又跑回来看同一个地方。
    """

    def __init__(self, default_radius: float = 0.4):
        # 默认拉黑半径（米）。在这个半径内的后续检测都会被忽略。
        self.default_radius = default_radius
        # 存储所有黑名单区域的列表
        self._zones: List[BlacklistZone] = []

    def add_zone(self, x: float, y: float, radius: float | None = None) -> None:
        """新增一个黑名单区域。
        
        参数:
            x, y: 误识别物体的平面坐标。
            radius: 拉黑半径。如果不传，则使用初始化时的默认值。
        """

        self._zones.append(
            BlacklistZone(x=x, y=y, radius=self.default_radius if radius is None else radius)
        )

    def contains(self, x: float, y: float) -> bool:
        """核心判断：判断一个新发现的候选点是否落在任何一个黑名单区域内。"""

        # any() 函数的意思是：只要列表中有一个为 True，结果就为 True。
        # 这里计算每个黑名单中心到新点的欧几里得距离（hypot），并与半径比较。
        return any(hypot(zone.x - x, zone.y - y) <= zone.radius for zone in self._zones)

    def zones(self) -> List[BlacklistZone]:
        """返回当前所有黑名单区域的快照（副本）。"""

        return list(self._zones)
