"""方案 B 下找物模块的纯 Python 核心实现。

这个包故意与 ROS 节点脚本分离：

- 纯 Python 层负责可测试的业务逻辑
- ROS 脚本层负责参数、消息、服务注册

这样即便感知侧还没写完，也可以先稳定接口和控制流程。
"""

from .anchor_catalog import AnchorPoint, NavAnchorCatalog
from .blacklist_manager import BlacklistManager
from .query_server import ObjectQueryService, QueryResult, DetectedCandidate

__all__ = [
    "AnchorPoint",
    "NavAnchorCatalog",
    "BlacklistManager",
    "ObjectQueryService",
    "QueryResult",
    "DetectedCandidate",
]
