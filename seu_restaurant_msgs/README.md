# *SEU RoboCup@Home Restaurant 2026*

东南大学 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_restaurant_msgs`

`seu_restaurant_msgs` 是 Restaurant 任务的公共接口包，统一定义状态机、导航、挥手检测、桌边交互和订单解析之间使用的 Action 协议。

## 用法命令

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_msgs
source devel/setup.bash

rosmsg show seu_restaurant_msgs/FindWavingCustomerAction
rosmsg show seu_restaurant_msgs/AlignAndTakeOrderAction
rosmsg show seu_restaurant_msgs/RestaurantNavAction
rosmsg show seu_restaurant_msgs/ParseRestaurantOrderAction
```

## 可能修改的地方

| 位置 | 说明 |
| --- | --- |
| `action/FindWavingCustomer.action` | 挥手找人接口 |
| `action/AlignAndTakeOrder.action` | 桌边对齐与接单接口 |
| `action/RestaurantNav.action` | 餐厅导航接口 |
| `action/ParseRestaurantOrder.action` | LLM 订单物品抽取接口 |

修改 `.action` 文件后，需要重新编译本包以及依赖这些接口的上层包。

## 技术栈 / Pipeline

- ROS ActionLib + `message_generation`
- 作为纯接口层，本包本身不单独启动运行节点
- 上游节点发送 Goal，下游节点回传 Result，所有模块按统一协议解耦协作

---
*SEU RoboCup Team 2026*
