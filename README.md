# *SEU RoboCup@Home Restaurant 2026*

东南大学 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## 项目概览

本仓库面向 RoboCup@Home Restaurant 赛题，核心流程覆盖挥手找客、导航接近、桌边点单、语音播报、订单解析和返回交付。

## 快速启动

先在工作区根目录完成编译：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

比赛运行时，建议按下面顺序启动：

终端 1：启动底盘与导航底层（上游 `tracer_nav`）

```bash
roslaunch tracer_nav nav_all.launch
```

终端 2：启动 `task_nav` 服务端

```bash
rosrun tracer_nav task_nav_server.py
```

终端 3：启动 Restaurant 全流程

```bash
roslaunch seu_restaurant_smach init.launch
```

## 模块说明

| 模块 | 作用 |
| --- | --- |
| `seu_restaurant_msgs` | Restaurant 任务公共 Action 接口定义 |
| `seu_restaurant_wave` | 挥手顾客搜索与目标坐标输出 |
| `seu_restaurant_interaction` | 桌边对齐、问单、ASR 与 LLM 联动 |
| `seu_restaurant_order_llm` | 英文订单句子的物品抽取 |
| `seu_voice` | 基于 Piper 的离线 TTS 服务 |
| `seu_speech_recognition` | 基于 Whisper 的语音识别服务 |
| `seu_nav` | 对接底层导航系统的餐厅导航 Action 包装层 |
| `seu_restaurant_smach` | 主状态机与全流程调度入口 |

## 常改位置

| 位置 | 说明 |
| --- | --- |
| `seu_restaurant_smach/launch/init.launch` | 全模块 bringup 入口 |
| `seu_restaurant_smach/scripts/restaurant_smach.py` | 主流程顺序、超时和播报文案 |
| `seu_restaurant_wave/scripts/wave_detector_core.py` | 挥手检测模型路径与阈值 |
| `seu_restaurant_order_llm/scripts/restaurant_order_llm_action_server.py` | LLM 模型与 prompt |
| `seu_speech_recognition/scripts/config.py` | 语音模型和音频路径 |
| `seu_voice/launch/voice_service.launch` | TTS 模型、播放命令和服务参数 |

## 技术栈 / Pipeline

- 基础框架：ROS Noetic、SMACH、ActionLib、ROS Service
- 视觉感知：RealSense + YOLO Pose
- 语音模块：Piper TTS + Whisper ASR
- 语言理解：Gemini 订单物品抽取
- 任务链路：挥手找客 -> 导航接近 -> 桌边对齐 -> 语音接单 -> LLM 解析 -> 回吧台播报 -> 返回顾客交付

各子模块的具体用法、变量位置和调试命令，见对应包内 `README.md`。

---
*SEU RoboCup Team 2026*
