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
    """管理候选点黑名单。"""

    def __init__(self, default_radius: float = 0.4):
        self.default_radius = default_radius
        self._zones: List[BlacklistZone] = []

    def add_zone(self, x: float, y: float, radius: float | None = None) -> None:
        """新增一个黑名单区域。"""

        self._zones.append(
            BlacklistZone(x=x, y=y, radius=self.default_radius if radius is None else radius)
        )

    def contains(self, x: float, y: float) -> bool:
        """判断一个候选点是否落在黑名单区域内。"""

        return any(hypot(zone.x - x, zone.y - y) <= zone.radius for zone in self._zones)

    def zones(self) -> List[BlacklistZone]:
        """返回当前黑名单区域快照。"""

        return list(self._zones)
