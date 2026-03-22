# *SEU RoboCup@Home Restaurant 2026*

东南大学自动化学院 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_nav`

`seu_nav` 是 Restaurant 任务的导航 Action 包装层，对上游提供 `restaurant_nav`，对下游继续调用 `tracer_nav` 的 `task_nav`，支持名字点和位姿两种目标模式。

## 用法命令

先启动底盘与下游导航系统：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch tracer_nav nav_all.launch
```

另开终端启动下游 `task_nav`：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun tracer_nav task_nav_server.py
```

再开终端启动 `seu_nav`：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_msgs seu_nav
source devel/setup.bash

rosrun seu_nav nav_action_server.py
```

## 可能修改的变量

| 位置 | 变量 | 说明 |
| --- | --- | --- |
| `scripts/nav_action_server.py` | `ACTION_NAME` `DOWNSTREAM_ACTION_NAME` | 上下游 Action 名称 |
| `scripts/nav_action_server.py` | `ROBOT_POSE_TOPIC` `GLOBAL_COSTMAP_TOPIC` `MAKE_PLAN_SERVICE_NAME` | 导航相关话题与服务 |
| `scripts/nav_action_server.py` | `DEFAULT_TIMEOUT_SEC` `DEFAULT_GOAL_SEARCH_RADIUS_M` `MAKE_PLAN_TOLERANCE_M` | 超时与可达点搜索参数 |
| `scripts/nav_action_server.py` | `WAYPOINTS_YAML_PATH` | 航点文件路径 |
| `config/waypoints.yaml` | `waypoints` 下各地点的 `nav_pose` 和 `pid_pose` | 场地标定数据 |

## 技术栈 / Pipeline

1. 上游状态机发送 `RestaurantNav` Action Goal。
2. 若是名字模式，则从 `config/waypoints.yaml` 读取 `nav_pose` 和 `pid_pose`。
3. 若是位姿模式，则结合 costmap 和 `make_plan` 计算最近可达点。
4. 将结果转换成下游 `task_nav` Goal 发送给 `tracer_nav`。
5. 返回导航成功或失败结果给上层状态机。

---
*SEU RoboCup Team 2026*
