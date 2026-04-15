#!/home/duo/anaconda3/envs/gpsr_whisper310/bin/python

import os
import sys
import rospy
import torch
import whisper
import pyaudio
import wave
from typing import Optional
from scipy.io import wavfile
import numpy as np
from seu_speech_recognition.srv import *
from datetime import datetime
from contextlib import contextmanager
import config
from seu_voice.srv import seu_voice as seu_voice_srv, seu_voiceRequest
# 定义音频文件路径
# 定义音频文件路径
AUDIO_OUTPUT_PATH = config.AUDIO_OUTPUT_PATH
PROCESSED_AUDIO_PATH = config.PROCESSED_AUDIO_PATH
# 需识别音频路径
wav_path = config.AUDIO_OUTPUT_PATH
_voice_client = None

# voice_service busy 重试策略：每 1s 查询一次，最多查询 5 次。
VOICE_BUSY_ERROR_CODE = 1002
VOICE_BUSY_QUERY_INTERVAL_SEC = 1.0
VOICE_BUSY_MAX_QUERY_COUNT = 5
VOICE_SERVICE_NAME = "voice_service"
PKG_NAME_SPEECH = "seu_speech_recognition"

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

def get_info(key):
    try:
        result = rospy.get_param(key)
    except:
        result = None
    return result


def _print_voice_busy_skip(pkg_name):
    """按需求打印红色提示：busy 超时后跳过语音。"""
    msg = f"[{pkg_name}]：voice service busy，skip speech。"
    sys.stderr.write(f"\033[31m{msg}\033[0m\n")
    sys.stderr.flush()


def _get_voice_client():
    """获取 voice_service 客户端单例。"""
    global _voice_client
    if _voice_client is None:
        _voice_client = rospy.ServiceProxy(VOICE_SERVICE_NAME, seu_voice_srv)
    return _voice_client


def _call_voice_service_blocking(text):
    """调用 voice_service 进行阻塞播报，并实现 busy 轮询重试逻辑。"""
    if text is None:
        return False
    text = str(text).strip()
    if not text:
        return False

    try:
        rospy.wait_for_service(VOICE_SERVICE_NAME, timeout=1.0)
    except rospy.ROSException as e:
        rospy.logerr("voice service unavailable: %s", e)
        return False

    client = _get_voice_client()
    req = seu_voiceRequest(text=text)

    def _call_once():
        try:
            return client.call(req)
        except rospy.ServiceException as call_error:
            rospy.logerr("voice service call failed: %s", call_error)
            return None

    # 第一次调用：服务端成功时会阻塞到播报结束。
    response = _call_once()
    if response is None:
        return False
    if response.state:
        return True
    if response.errorcode != VOICE_BUSY_ERROR_CODE:
        rospy.logerr("voice service failed: errorcode=%s, errormsg=%s",
                     response.errorcode, response.errormsg)
        return False

    # busy 时每 1s 查询一次，最多查询 5 次。
    for _ in range(VOICE_BUSY_MAX_QUERY_COUNT):
        rospy.sleep(VOICE_BUSY_QUERY_INTERVAL_SEC)
        response = _call_once()
        if response is None:
            return False
        if response.state:
            return True
        if response.errorcode != VOICE_BUSY_ERROR_CODE:
            rospy.logerr("voice service failed after busy retry: errorcode=%s, errormsg=%s",
                         response.errorcode, response.errormsg)
            return False

    _print_voice_busy_skip(PKG_NAME_SPEECH)
    return False

def speech_to_text(data):
    # 通过 voice_service 阻塞播报；失败（含 busy 超时）时按策略跳过。
    _call_voice_service_blocking(data)
    log_file = get_info('log')
    if not log_file:
        return
    # 获取当前时间戳
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
    # 确保日志目录存在
    log_dir = os.path.dirname(log_file)
    if log_dir and not os.path.exists(log_dir):
        os.makedirs(log_dir)
    # 追加写入日志文件
    with open(f"{log_file}.txt", "a", encoding="utf-8") as f:
        f.write(f"[{timestamp}] {data}\n")

class SimpleSpeechRecognizer:
    """简化语音处理类：提供录音、降噪、识别三步功能"""

    def __init__(self,
                 model_name: str = "base",
                 language: str = "en"):
        # 自动选择设备：优先 GPU
        self.device = "cpu" if torch.cuda.is_available() else "cpu"
        self.language = language
        self.model_name = model_name

        # 加载 Whisper 模型
        print(f"加载 Whisper 模型: {model_name}")
        if self.device == "cuda":
            try:
                print(f"使用 GPU 推理: {torch.cuda.get_device_name(0)}")
            except Exception:
                print("使用 GPU 推理: cuda:0")
        else:
            print("未检测到可用 CUDA，回退到 CPU 推理")
        self.model = whisper.load_model(model_name, device=self.device)

    def record_audio(self,
                     output_audio_path: str = AUDIO_OUTPUT_PATH,
                     max_duration: int = 5,
                     silence_timeout: int = 3) -> str:
        """从麦克风录音并保存为 wav 文件"""
        print("开始录音...")

        # 使用 pyaudio 进行录音
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
            # CPU 下显式禁用 fp16，避免 Whisper 重复警告
            result = self.model.transcribe(
                audio_path,
                language=language or self.language,
                fp16=(self.device == "cuda")
            )
            text = result.get("text", "").strip()
            print("识别完成。")
            return text
        except Exception as e:
            print(f"识别失败: {e}")
            return ""

    def recognition(self,req = None,path = wav_path):
        if req is not None:
            if req.task == "command":
                speech_to_text("please say the command")
            elif req.task == "answer":
                speech_to_text("please ask the question")
        try:
            recorded_path = self.record_audio()
            # 步骤 2：降噪（可选）
            # denoised_path = recognizer.denoise_audio(recorded_path)
            word = self.transcribe_audio(recorded_path)
            word = word.lower()
            print("\n识别文本：\n", word)
            # 识别为空
            if len(word) == 0:
                state = False
                errorcode = 1
                errormsg = 'empty'
                rospy.logerr('errormsg: %s', errormsg)
            # 识别有效
            else:
                state = True
                errorcode = 0
                errormsg = 'success'
                rospy.loginfo('word: %s', word)
        # 识别报错
        except AttributeError:
            word = ''
            state = False
            errorcode = -1
            errormsg = 'failure'
            rospy.logerr('errormsg: %s', errormsg)
        return seu_speech_recognitionResponse(state, errorcode, word, errormsg)


recognizer = SimpleSpeechRecognizer(
    model_name=config.MODEL_PATH,
    language="en")
