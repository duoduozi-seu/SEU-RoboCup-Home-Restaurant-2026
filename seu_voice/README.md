# *SEU RoboCup@Home Restaurant 2026*

东南大学 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_voice`

`seu_voice` 是 Restaurant 任务的离线语音播报服务包，基于 Piper TTS 提供阻塞式 ROS Service，适合问单提示、吧台复述和任务结束播报。

## 用法命令

首次使用前，建议先安装 `piper-tts` 和 `alsa-utils`。

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_voice
source devel/setup.bash

roslaunch seu_voice voice_service.launch
```

另开终端测试服务：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosservice call /voice_service "text: 'Hello, I am ready.'"
```

也可以用示例客户端测试：

```bash
rosrun seu_voice voice_client_demo.py "Hello, I am ready."
```

## 可能修改的变量

| 位置 | 变量 | 说明 |
| --- | --- | --- |
| `launch/voice_service.launch` | `service_name` `model_path` `config_path` `player_cmd` `tmp_dir` | 启动参数入口 |
| `scripts/voice_service.py` | `ERR_EMPTY_TEXT` `ERR_BUSY` `ERR_SYNTH_FAILED` `ERR_PLAY_FAILED` `ERR_MODEL_LOAD_FAILED` | 服务返回码 |
| `models/` | `en_US-lessac-medium.onnx` `en_US-lessac-medium.onnx.json` | 默认语音模型与配置 |

## 技术栈 / Pipeline

1. 上游模块调用 `/voice_service` 发送文本。
2. Piper 将文本合成为临时 WAV 文件。
3. 本地播放器命令默认使用 `aplay -q` 立即播报。
4. 服务在播放结束后返回状态、耗时和错误码。

---
*SEU RoboCup Team 2026*
