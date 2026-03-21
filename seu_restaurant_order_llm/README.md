# *SEU RoboCup@Home Restaurant 2026*
---
东南大学自动化学院 RoboCup@Home Restaurant 赛题代码

<!-- <div align="center">
  <span style="font-family: 'Times New Roman', 'SimSun', serif; font-size: 20px;">周艺舵</span><br/>
  <span style="font-family: 'Times New Roman', 'SimSun', serif; font-size: 16px;">东南大学自动化学院 (School of Automation, Southeast University)</span>
</div>

**Environment**

- Ubuntu 20.04
- ROS Noetic
- Python 3.10 -->

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

# seu_restaurant_order_llm

## 1. 包用途

这个包用于给 Restaurant 题提供一个独立的 LLM 订单解析 action 节点。

输入：

- 一整句英文订单文本，例如：`i want a bottle of cola, a bag of chips and a bottle of mineral water`

输出：

- `string[] items`，例如：`["cola", "chips", "mineral water"]`

当前实现特点：

- 通过 `actionlib` 与上游 SMACH 通信；
- 不做硬编码物品映射关系；
- 主要依靠规则 prompt + few-shot prompt 做关键物品短语抽取；
- Python 端只做最基础的大小写、空格清洗和 JSON 数组解析。

## 2. 包内关键文件

- `action/ParseRestaurantOrder.action`
  - action 接口定义。
- `scripts/gemini_api_core.py`
  - Gemini HTTP 基础调用封装。
- `scripts/restaurant_order_llm_action_server.py`
  - ROS 核心业务代码，包含 action server、prompt 常量和解析逻辑。
- `test/test_order_parser_core.py`
  - 不依赖网络的核心单元测试。
- `test/test.py`
  - actionlib 客户端联调脚本。

## 3. 你最可能会改的常量

这些常量都集中在 `scripts/restaurant_order_llm_action_server.py`：

- `ACTION_NAME`
  - action 名字。
- `GEMINI_MODEL`
  - 当前使用的 Gemini 模型。
- `MAX_RETRY_TIMES`
  - 接口失败或格式错误时的最大重试次数。
- `ORDER_PARSE_RULE_PROMPT`
  - 规则约束 prompt。
- `ORDER_PARSE_FEW_SHOT_PROMPT`
  - few-shot 示例 prompt。
- `RETRY_HINT_PROMPT`
  - 二次重试时追加的格式提醒。

Gemini 网关和 key 集中在 `scripts/gemini_api_core.py`：

- `GEMINI_API_KEY`
- `GEMINI_BASE_URL`
- `GEMINI_API_VERSION`
- `HTTP_TIMEOUT_SEC`

## 4. 构建命令

在工作区根目录执行：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_order_llm
```

## 5. 启动服务端

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun seu_restaurant_order_llm restaurant_order_llm_action_server.py
```

## 6. 运行单元测试

```bash
source /opt/ros/noetic/setup.bash
python3 /home/duo/GPSR_ws_zyd/catkin_robocup_2/src/Restaurant/src/seu_restaurant_order_llm/test/test_order_parser_core.py
```

## 7. 运行 actionlib 客户端测试

先保证服务端已经启动，再执行：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/Restaurant/src/seu_restaurant_order_llm/test/test.py
```

默认测试句子写在：

- `test/test.py` 里的 `TEST_ORDER_TEXT`

---
*SEU RoboCup Team 2026*