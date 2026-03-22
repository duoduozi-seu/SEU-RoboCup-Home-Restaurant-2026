# *SEU RoboCup@Home Restaurant 2026*

东南大学自动化学院 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_restaurant_order_llm`

`seu_restaurant_order_llm` 提供英文点单句子的物品抽取 Action 服务，把整句订单解析成可供状态机直接使用的 `string[] items`。

## 用法命令

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_msgs seu_restaurant_order_llm
source devel/setup.bash

rosrun seu_restaurant_order_llm restaurant_order_llm_action_server.py
```

另开终端执行 Action 客户端联调：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/Restaurant/src/seu_restaurant_order_llm/test/test.py
```

如果只想跑纯解析单元测试，可执行：

```bash
python3 /home/duo/GPSR_ws_zyd/catkin_robocup_2/src/Restaurant/src/seu_restaurant_order_llm/test/test_order_parser_core.py
```

## 可能修改的变量

| 位置 | 变量 | 说明 |
| --- | --- | --- |
| `scripts/restaurant_order_llm_action_server.py` | `ACTION_NAME` | Action 名称 |
| `scripts/restaurant_order_llm_action_server.py` | `GEMINI_MODEL` `MAX_RETRY_TIMES` | 模型与重试次数 |
| `scripts/restaurant_order_llm_action_server.py` | `ORDER_PARSE_RULE_PROMPT` `ORDER_PARSE_FEW_SHOT_PROMPT` `RETRY_HINT_PROMPT` | 订单抽取提示词 |
| `scripts/gemini_api_core.py` | `GEMINI_API_KEY` `GEMINI_BASE_URL` `GEMINI_API_VERSION` `HTTP_TIMEOUT_SEC` | 网关、密钥与超时 |
| `test/test.py` | `TEST_ORDER_TEXT` | 联调时的默认测试语句 |

## 技术栈 / Pipeline

1. 上游发送 `order_text` 到 `parse_restaurant_order` Action。
2. 节点拼接规则 prompt 和 few-shot prompt。
3. Gemini 返回 JSON 数组形式的物品列表。
4. Python 端做轻量清洗、解析和重试控制。
5. 最终输出 `items`，供交互模块和状态机继续使用。

---
*SEU RoboCup Team 2026*
