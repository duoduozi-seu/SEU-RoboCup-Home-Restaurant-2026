# seu_voice

ROS1 (Noetic) TTS service package based on Piper.

## Features

- Blocking ROS service call.
- Each request synthesizes speech immediately and plays it immediately.
- Service returns only after playback is finished.
- Busy requests are rejected with explicit error code.

## Service API

Service name (default): `/voice_service`

Service type: `seu_voice/seu_voice`

```srv
string text
---
bool state
int64 errorcode
float32 synth_time_sec
float32 play_time_sec
float32 total_time_sec
string errormsg
```

Error codes:

- `0`: success
- `1001`: empty text
- `1002`: busy
- `1003`: synthesis failed
- `1004`: playback failed
- `1005`: model/config load failed

## Runtime Params

- `~service_name` (default: `voice_service`)
- `~model_path`
- `~config_path`
- `~player_cmd` (default: `aplay -q`)
- `~tmp_dir` (default: `/tmp/seu_voice`)

## Environment Setup (manual)

```bash
cd /home/duo/GPSR_ws_zyd/GPSR_code
# conda activate GPSR

python3 -m pip install -U pip
python3 -m pip install piper-tts

sudo apt-get update
sudo apt-get install -y alsa-utils
```

## Download Default Model

```bash
mkdir -p /home/duo/GPSR_ws_zyd/GPSR_code/src/seu_voice/models
cd /home/duo/GPSR_ws_zyd/GPSR_code/src/seu_voice/models
python3 -m piper.download_voices en_US-lessac-medium --data-dir .

find . -name 'en_US-lessac-medium.onnx*'
```

## Build and Run

```bash
cd /home/duo/GPSR_ws_zyd/GPSR_code
catkin_make --pkg seu_voice
source devel/setup.bash
roslaunch seu_voice voice_service.launch
```

Note:

- `voice_service.launch` uses `type="scripts/voice_service.py"` so it follows your current shell `python3` (for conda environments).
- If you run `rosrun seu_voice voice_service.py`, catkin may use `/usr/bin/python3` wrapper in `devel/lib/seu_voice/`, and that interpreter also needs `piper-tts`.

## Test

```bash
rosservice call /voice_service "text: 'Hello, I am ready.'"
```

or:

```bash
rosrun seu_voice voice_client_demo.py "Hello, I am ready."
```
