# *SEU RoboCup@Home Restaurant 2026*
---
Codebase for the SEU RoboCup@Home Restaurant task.

- Author: Yiduo Zhou, Department of Robotics Engineering, School of Automation, Southeast University
- Development environment: Ubuntu 20.04 + ROS Noetic

## Overview

This repository is designed for the RoboCup@Home Restaurant task. The core workflow covers waving customer detection, navigation and approach, tableside ordering, voice announcements, order parsing, and return delivery.

## News

- We won the RoboCup@Home China Open 2026 championship in Shunyi, Beijing.

## Quick Start

Build the workspace from the workspace root:

```bash
cd /home/duo/GPSR_ws_zyd/catkin_robocup_2
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

During competition runs, start the system in the following order:

Terminal 1: start the chassis and low-level navigation stack from upstream `tracer_nav`.

```bash
roslaunch tracer_nav nav_all.launch
```

Terminal 2: start the `task_nav` server.

```bash
rosrun tracer_nav task_nav_server.py
```

Terminal 3: start the full Restaurant pipeline.

```bash
roslaunch seu_restaurant_smach init.launch
```

## Modules

| Module | Description |
| --- | --- |
| `seu_restaurant_msgs` | Shared Action interface definitions for the Restaurant task |
| `seu_restaurant_wave` | Waving customer search and target coordinate output |
| `seu_restaurant_interaction` | Tableside alignment, order taking, ASR, and LLM integration |
| `seu_restaurant_order_llm` | Item extraction from English order sentences |
| `seu_voice` | Offline TTS service based on Piper |
| `seu_speech_recognition` | Speech recognition service based on Whisper |
| `seu_nav` | Restaurant navigation Action wrapper connected to the low-level navigation system |
| `seu_restaurant_smach` | Main state machine and full workflow scheduler |

## Commonly Edited Files

| Path | Description |
| --- | --- |
| `seu_restaurant_smach/launch/init.launch` | Bringup entry point for all modules |
| `seu_restaurant_smach/scripts/restaurant_smach.py` | Main workflow order, timeouts, and spoken prompts |
| `seu_restaurant_wave/scripts/wave_detector_core.py` | Waving detection model path and thresholds |
| `seu_restaurant_order_llm/scripts/restaurant_order_llm_action_server.py` | LLM model and prompt |
| `seu_speech_recognition/scripts/config.py` | Speech model and audio paths |
| `seu_voice/launch/voice_service.launch` | TTS model, playback command, and service parameters |

## Tech Stack / Pipeline

- Core framework: ROS Noetic, SMACH, ActionLib, ROS Service
- Visual perception: RealSense + YOLO Pose
- Speech modules: Piper TTS + Whisper ASR
- Language understanding: Gemini-based order item extraction
- Task pipeline: waving customer detection -> navigation and approach -> tableside alignment -> voice order taking -> LLM parsing -> return to the bar counter for announcement -> return to the customer for delivery

For detailed usage, variable locations, and debugging commands, see the `README.md` files in the corresponding packages.

---
*SEU RoboCup Team 2026*
