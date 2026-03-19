# 2026 SEU RoboCup@Home Restaurant Code

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

## 工作空间构建

```bash
cd ~/GPSR_ws_zyd/GPSR_code
catkin_make
source devel/setup.bash
```

## 模块 1：`seu_voice`（离线语音合成，Piper）

### 环境配置

```bash
# 建议在项目使用的 Python/Conda 环境中执行
python3 -m pip install -U pip
python3 -m pip install piper-tts

sudo apt-get update
sudo apt-get install -y alsa-utils
```

模型文件需存在于：

- `src/seu_voice/models/en_US-lessac-medium.onnx`
- `src/seu_voice/models/en_US-lessac-medium.onnx.json`

### 启动与测试

```bash
source ~/GPSR_ws_zyd/GPSR_code/devel/setup.bash
roslaunch seu_voice voice_service.launch
```

另开终端测试：

```bash
source ~/GPSR_ws_zyd/GPSR_code/devel/setup.bash
rosservice call /voice_service "text: 'hello, i am ready'"
```

## 模块 2：`seu_speech_recognition`（Whisper 语音识别）

### 环境配置

已用 `conda list` + 环境内 `pip list` 核实当前可用环境为 `gpsr_whisper310`（Python 3.10.19）。

```bash
# 核对命令
conda activate gpsr_whisper310
conda list | egrep "python|faster-whisper|ctranslate2|openai-whisper|torch|pyaudio|scipy|numpy"
pip list   | egrep "faster-whisper|ctranslate2|openai-whisper|torch|PyAudio|scipy|numpy"
```

当前环境关键版本（实测）：

- `python`: `3.10.19`
- `faster-whisper`: `1.2.1`
- `ctranslate2`: `4.7.1`
- `openai-whisper`: `20250625`
- `torch/torchaudio/torchvision`: `2.4.0+cu121 / 2.4.0+cu121 / 0.19.0+cu121`
- `PyAudio`: `0.2.14`
- `scipy`: `1.15.3`
- `numpy`: `2.2.6`

如需按当前环境重装关键依赖：

```bash
conda activate gpsr_whisper310
python3 -m pip install -U pip
python3 -m pip install \
  faster-whisper==1.2.1 ctranslate2==4.7.1 \
  openai-whisper==20250625 \
  PyAudio==0.2.14 scipy==1.15.3 numpy==2.2.6
```

模型默认路径（见 `scripts/config.py`）：

- `src/seu_speech_recognition/scripts/large-v3-turbo.pt`

该模块会调用 `voice_service` 进行提示音播报，建议先启动 `seu_voice`。

### 启动与测试

```bash
source ~/GPSR_ws_zyd/GPSR_code/devel/setup.bash
rosrun seu_speech_recognition speech_service.py
```

另开终端测试：

```bash
source ~/GPSR_ws_zyd/GPSR_code/devel/setup.bash
rosservice call /speech_recognition_service "enable: false
task: 'command'"
```

参数说明：

- `enable`：当前代码中为保留字段，`true/false` 目前均不改变识别流程。
- `task`：用于提示语选择，`command` 会播报 `please say the command`，`answer` 会播报 `please ask the question`。

## 当前代码说明（与旧版对比）

- `seu_voice` 已切换为 Piper TTS 服务，阻塞式返回，含 busy/errorcode 机制。
- `seu_speech_recognition` 当前运行环境为 Python 3.10 + faster-whisper（环境已就绪）。
- 当前仓库 `my_whisper.py` 代码仍为 `openai-whisper` 调用接口（录音-识别），并与 `seu_voice` 服务联动。
- `seu_speech_recognition/launch/run.launch` 仍引用 `run.sh`，但仓库中无该文件；请按上面的 `rosrun` 命令启动。
