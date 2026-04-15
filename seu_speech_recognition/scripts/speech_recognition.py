import os
import sys
import torch
import whisper
import pyaudio
import wave
from typing import Optional
from scipy.io import wavfile
import numpy as np
from contextlib import contextmanager
import config

# 定义音频文件路径
AUDIO_OUTPUT_PATH = config.AUDIO_OUTPUT_PATH
PROCESSED_AUDIO_PATH = config.PROCESSED_AUDIO_PATH


@contextmanager
def ignore_stderr():
    """临时重定向 stderr 以消除 ALSA/PortAudio 的干扰信息"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    sys.stderr.flush()
    os.dup2(devnull, 2)
    os.close(devnull)
    try:
        yield
    finally:
        os.dup2(old_stderr, 2)
        os.close(old_stderr)


class SimpleSpeechRecognizer:
    """简化语音处理类：提供录音、降噪、识别三步功能"""

    def __init__(self,
                 model_name: str = "base",
                 language: str = "en"):
        # 自动选择设备
        self.device = "cuda"
        self.language = language
        self.model_name = model_name

        # 加载 Whisper 模型
        print(f"加载 Whisper 模型: {model_name}")
        self.model = whisper.load_model(model_name, device=self.device)

    def record_audio(self,
                     output_audio_path: str = AUDIO_OUTPUT_PATH,
                     max_duration: int = 5,
                     silence_timeout: int = 3) -> str:
        """从麦克风录音并保存为 wav 文件"""
        print("开始录音...")

        # 使用 ignore_stderr 屏蔽 ALSA 报错提示
        with ignore_stderr():
            p = pyaudio.PyAudio()

        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=16000,
                        input=True,
                        frames_per_buffer=1024)

        frames = []
        print("录音开始，按 Ctrl+C 停止录音...")
        try:
            for _ in range(0, int(16000 / 1024 * max_duration)):
                data = stream.read(1024)
                frames.append(data)
        except KeyboardInterrupt:
            print("录音结束。")

        # 保存为 wav 文件
        stream.stop_stream()
        stream.close()
        p.terminate()

        # 保存音频为 wav 格式
        with wave.open(output_audio_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b''.join(frames))

        print(f"录音完成: {output_audio_path}")
        return output_audio_path

    def denoise_audio(self,
                      input_path: str = AUDIO_OUTPUT_PATH,
                      output_path: str = PROCESSED_AUDIO_PATH) -> str:
        """应用简单的降噪（这里用numpy进行简单的低通滤波降噪）"""
        print("应用降噪中...")

        # 读取音频文件
        sample_rate, audio_data = wavfile.read(input_path)

        # 简单降噪：用低通滤波器减少高频噪声
        from scipy.signal import butter, filtfilt

        def butter_lowpass(cutoff, fs, order=5):
            nyquist = 0.5 * fs
            normal_cutoff = cutoff / nyquist
            b, a = butter(order, normal_cutoff, btype='low', analog=False)
            return b, a

        def butter_lowpass_filter(data, cutoff, fs, order=5):
            b, a = butter_lowpass(cutoff, fs, order)
            y = filtfilt(b, a, data)
            return y

        # 假设降噪只做低通滤波
        cutoff = 5000.0  # 设置低通滤波的截止频率
        filtered_audio = butter_lowpass_filter(audio_data, cutoff, sample_rate)

        # 保存降噪后的音频文件
        wavfile.write(output_path, sample_rate, filtered_audio.astype(np.int16))

        print(f"降噪完成: {output_path}")
        return output_path

    def transcribe_audio(self,
                         audio_path: str,
                         language: Optional[str] = None) -> str:
        """将音频转录为文本"""
        print("开始语音识别...")
        try:
            result = self.model.transcribe(audio_path, language=language or self.language)
            text = result.get("text", "").strip()
            print("识别完成。")
            return text
        except Exception as e:
            print(f"识别失败: {e}")
            return ""


if __name__ == "__main__":
    recognizer = SimpleSpeechRecognizer(model_name=config.MODEL_PATH, language="en")

    try:
        # 步骤 1：录音
        recorded_path = recognizer.record_audio()
        recorded_path = config.AUDIO_OUTPUT_PATH
        # 步骤 2：降噪（可选）
        #denoised_path = recognizer.denoise_audio(recorded_path)

        # 步骤 3：识别
        result_text = recognizer.transcribe_audio(recorded_path)
        print("\n识别文本：\n", result_text)

    except Exception as err:
        print(f"出错：{err}")
