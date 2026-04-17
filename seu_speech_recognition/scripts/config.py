#!/usr/bin/env python
# -*- coding:UTF-8 -*-
import os

# ==============================================================================
# 路径配置文件 (Configuration File)
# ==============================================================================
# 
# 本文件统一管理所有硬编码的路径。
# 所有路径均基于当前文件所在目录 (scripts) 动态生成，以确保可移植性。
#

# 获取当前脚本所在目录 (/src/seu_speech_recognition/scripts)
SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------------------
# 音频路径配置 (Audio Paths)
# ------------------------------------------------------------------------------

# 1. 录音文件保存目录
#    文件: my_whisper.py, speech_recogition.py
#    作用: 存放录制的 .wav 音频文件的文件夹
WAV_DIR = os.path.join(SCRIPTS_DIR, "wav")

# 确保目录存在
if not os.path.exists(WAV_DIR):
    os.makedirs(WAV_DIR)

# 2. 默认录音文件路径
#    文件: my_whisper.py (变量: AUDIO_OUTPUT_PATH, wav_path)
#    文件: speech_recogition.py (变量: AUDIO_OUTPUT_PATH)
#    作用: 麦克风录音后保存的完整文件路径
AUDIO_OUTPUT_PATH = os.path.join(WAV_DIR, "recording.wav")

# 3. 降噪处理后的音频路径
#    文件: my_whisper.py (变量: PROCESSED_AUDIO_PATH)
#    文件: speech_recogition.py (变量: PROCESSED_AUDIO_PATH)
#    作用: 保存经过降噪处理后的临时音频文件名
PROCESSED_AUDIO_PATH = "denoised_audio.wav"


# ------------------------------------------------------------------------------
# 模型路径配置 (Model Paths)
# ------------------------------------------------------------------------------

# 1. Whisper 模型路径
#    faster-whisper 推荐填模型目录（snapshot_download / hf download 的目标目录）
#    当前指向：~/GPSR_ws_zyd/models/faster-whisper-large-v3
# MODEL_PATH = "/home/duo/GPSR_ws_zyd/models/faster-whisper-large-v3"
MODEL_PATH = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/Restaurant/src/seu_speech_recognition/scripts/small.en.pt"