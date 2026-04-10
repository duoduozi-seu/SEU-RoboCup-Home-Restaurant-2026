# *SEU RoboCup@Home Restaurant 2026*

东南大学 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_restaurant_smach`

`seu_restaurant_smach` 是 Restaurant 任务的主状态机包，负责把挥手检测、导航、桌边交互、订单播报和回送流程串成一条完整的双人服务任务链。

## 用法命令

推荐直接使用 `init.launch` 一次启动各个核心模块：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_smach
source devel/setup.bash

roslaunch seu_restaurant_smach init.launch
```

如果相关服务都已经启动，也可以单独运行主状态机：

```bash
rosrun seu_restaurant_smach restaurant_smach.py
```

## 可能修改的变量

| 位置 | 变量 | 说明 |
| --- | --- | --- |
| `scripts/restaurant_smach.py` | `WAVE_ACTION_NAME` `NAV_ACTION_NAME` `INTERACTION_ACTION_NAME` | 各 Action / Service 名称 |
| `scripts/restaurant_smach.py` | `WAVE_TIMEOUT_SEC` `NAV_TIMEOUT_SEC` `ALIGN_TIMEOUT_SEC` `DIALOG_TIMEOUT_SEC` | 流程超时设置 |
| `scripts/restaurant_smach.py` | `STAND_OFF_DISTANCE_M` `ORDER_MAX_ROUNDS` | 接近顾客距离与问单轮数 |
| `scripts/restaurant_smach.py` | `ASK_ORDER_TEXT` `ORDER_REPEAT_PREFIX` `FINAL_DELIVER_TEXT` | 播报文案 |
| `launch/init.launch` | 各模块启动顺序和启动参数 | 全流程 bringup 入口 |

## 技术栈 / Pipeline

1. 记录初始吧台位姿，并保存一个面朝吧台的回归位姿。
2. 调用 `seu_restaurant_wave` 一次寻找最多两位挥手顾客。
3. 顶层状态机维护顾客队列，循环调用“单人服务子状态机”。
4. 单人服务子状态机完成接近顾客、接单、回吧台播报、回顾客送餐。
5. 每服务完一位顾客后，直接导航回吧台初始坐标与背对吧台的初始朝向。
6. 共完成两位顾客后结束任务。

---
*SEU RoboCup Team 2026*
