# *SEU RoboCup@Home Restaurant 2026*

东南大学 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_restaurant_interaction`

`seu_restaurant_interaction` 负责顾客桌边的局部对齐、语音问单、yes/no 确认，以及把确认后的订单文本交给 LLM 做物品抽取。

## 用法命令

建议先保证 `seu_voice`、`seu_speech_recognition`、`seu_restaurant_order_llm` 都已经启动。

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_msgs seu_restaurant_interaction
source devel/setup.bash

rosrun seu_restaurant_interaction align_and_take_order_action_server.py
```

如需跑已有测试：

```bash
python3 /home/duo/GPSR_ws_zyd/catkin_robocup_2/src/Restaurant/src/seu_restaurant_interaction/tests/test_confirmation_parser.py
python3 /home/duo/GPSR_ws_zyd/catkin_robocup_2/src/Restaurant/src/seu_restaurant_interaction/tests/test_align_and_take_order_llm_flow.py
```

## 可能修改的变量

| 位置 | 变量 | 说明 |
| --- | --- | --- |
| `scripts/align_and_take_order_action_server.py` | `ACTION_NAME` `ORDER_PARSER_ACTION_NAME` | 交互 Action 与订单解析 Action 名称 |
| `scripts/align_and_take_order_action_server.py` | `VOICE_SERVICE_NAME` `SPEECH_SERVICE_NAME` | TTS / ASR 服务名 |
| `scripts/align_and_take_order_action_server.py` | `MODEL_PATH` | 局部对齐使用的姿态模型路径 |
| `scripts/align_and_take_order_action_server.py` | `TARGET_SERVICE_DISTANCE_M` `CENTER_TOLERANCE_PX` `DISTANCE_TOLERANCE_M` | 对齐距离与误差阈值 |
| `scripts/align_and_take_order_action_server.py` | `ANGULAR_SPEED` `LINEAR_SPEED` | 底盘微调速度 |

## 技术栈 / Pipeline

1. YOLO Pose + RealSense 找到桌边最近顾客。
2. 机器人原地旋转和前后微调，完成局部对齐。
3. 调用 `seu_voice` 播放问单与确认提示语。
4. 调用 `seu_speech_recognition` 获取顾客原始订单文本。
5. 顾客确认后，再调用 `seu_restaurant_order_llm` 提取 `orderlist`。

---
*SEU RoboCup Team 2026*
