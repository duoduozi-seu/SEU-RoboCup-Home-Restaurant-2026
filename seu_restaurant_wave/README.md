# *SEU RoboCup@Home Restaurant 2026*

东南大学 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_restaurant_wave`

`seu_restaurant_wave` 负责原地旋转搜索挥手顾客，并把最多两个最近目标转换到 `map` 坐标系后通过 Action 返回给上层状态机。

## 用法命令

推荐在已经配置好 RealSense、PyTorch、Ultralytics 的感知环境中运行：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_msgs seu_restaurant_wave
source devel/setup.bash

rosrun seu_restaurant_wave find_waving_customer_action_server.py
```

另开终端执行测试客户端：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/Restaurant/src/seu_restaurant_wave/client_test/wave_test_client.py
```

## 可能修改的变量

| 位置 | 变量 | 说明 |
| --- | --- | --- |
| `scripts/find_waving_customer_action_server.py` | `ACTION_NAME` | Action 名称 |
| `scripts/find_waving_customer_action_server.py` | `ROBOT_POSE_TOPIC` `CMD_VEL_TOPIC` | 机器人位姿与底盘控制话题 |
| `scripts/find_waving_customer_action_server.py` | `DEFAULT_CAMERA_FRAME` `DEFAULT_RATE_HZ` | 相机坐标系与处理频率 |
| `scripts/wave_detector_core.py` | `POSE_MODEL_PATH` | 姿态模型路径 |
| `scripts/wave_detector_core.py` | `POSE_CONF` `POSE_IOU` `WAVE_HISTORY` `SIMPLE_RAISE_REQUIRED_FRAMES` | 检测与挥手判定阈值 |

## 技术栈 / Pipeline

1. RealSense 采集彩色图和深度图。
2. YOLO Pose 提取人体关键点。
3. 时序规则判断是否为挥手目标，并按距离排序筛出最近的两位顾客。
4. 将相机坐标转换到 `map` 坐标系，返回 `person_points_map[]`。
5. 同时发布调试图像和 RViz marker，方便联调。

---
*SEU RoboCup Team 2026*
