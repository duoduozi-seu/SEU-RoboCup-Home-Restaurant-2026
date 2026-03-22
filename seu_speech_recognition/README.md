# *SEU RoboCup@Home Restaurant 2026*

东南大学 RoboCup@Home Restaurant 赛题代码

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic

## `seu_speech_recognition`

`seu_speech_recognition` 是 Restaurant 任务的语音识别服务包，负责麦克风录音、Whisper 转写，并把结果通过 ROS Service 返回给桌边交互模块。

## 用法命令

建议先启动 `seu_voice`，这样服务在录音前可以先播报提示语。

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_speech_recognition
source devel/setup.bash

LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libtiff.so.5 rosrun seu_speech_recognition speech_service.py
```

另开终端测试服务：

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosservice call /speech_recognition_service "{enable: false, task: 'command'}"
```

## 可能修改的变量

| 位置 | 变量 | 说明 |
| --- | --- | --- |
| `scripts/config.py` | `WAV_DIR` `AUDIO_OUTPUT_PATH` `PROCESSED_AUDIO_PATH` | 录音与中间文件路径 |
| `scripts/config.py` | `MODEL_PATH` | Whisper 模型路径 |
| `scripts/my_whisper.py` | `VOICE_SERVICE_NAME` `VOICE_BUSY_MAX_QUERY_COUNT` | 语音提示与 busy 重试策略 |
| `scripts/my_whisper.py` | `record_audio()` 默认参数 | 录音时长、采样率等行为入口 |
| `scripts/speech_service.py` | `speech_recognition_service` | 服务名 |

## 技术栈 / Pipeline

1. `speech_service.py` 注册 `/speech_recognition_service`。
2. `my_whisper.py` 先调用 `seu_voice` 播放提示语。
3. PyAudio 从麦克风录音并保存到 `wav/` 目录。
4. Whisper 模型完成英文转写。
5. 服务返回 `word`、`state` 和 `errorcode` 给上层模块。

---
*SEU RoboCup Team 2026*
