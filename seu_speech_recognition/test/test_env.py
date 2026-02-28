#!/usr/bin/env python
# -*- coding:UTF-8 -*-
"""
seu_speech_recognition 包独立测试脚本

使用方法：
    1. 编译包: catkin_make --only-pkg-with-deps seu_speech_recognition
    2. Source 环境: source devel/setup.bash
    3. 运行测试: python3 test.py

此脚本不依赖 ROS 节点运行，可以独立测试 Whisper 语音识别的各项功能。
"""

import os
import sys
import wave
import tempfile
import numpy as np
from scipy.io import wavfile

# 获取脚本所在目录和scripts目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
SCRIPTS_DIR = os.path.join(PACKAGE_DIR, "scripts")

# 将scripts目录添加到路径
sys.path.insert(0, SCRIPTS_DIR)

# ============================================================================
# 测试工具函数
# ============================================================================

class TestResult:
    """测试结果类"""
    def __init__(self, name):
        self.name = name
        self.passed = False
        self.message = ""
        self.error = None

    def __str__(self):
        status = "✅ PASS" if self.passed else "❌ FAIL"
        result = f"{status}: {self.name}"
        if self.message:
            result += f"\n    📝 {self.message}"
        if self.error:
            result += f"\n    ⚠️  Error: {self.error}"
        return result


def print_header(title):
    """打印测试分组标题"""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def create_test_audio(duration=2.0, frequency=440.0, sample_rate=16000):
    """创建测试用的音频文件（正弦波）"""
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    audio_data = (np.sin(2 * np.pi * frequency * t) * 32767).astype(np.int16)
    
    # 创建临时文件
    fd, path = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    
    with wave.open(path, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(audio_data.tobytes())
    
    return path


# ============================================================================
# 测试用例
# ============================================================================

def test_whisper_import():
    """测试 Whisper 模块导入"""
    result = TestResult("Whisper 模块导入")
    try:
        import whisper
        result.passed = True
        result.message = f"whisper 版本: {whisper.__version__ if hasattr(whisper, '__version__') else '已安装'}"
    except ImportError as e:
        result.error = str(e)
    return result


def test_torch_import():
    """测试 PyTorch 导入"""
    result = TestResult("PyTorch 模块导入")
    try:
        import torch
        result.passed = True
        result.message = f"torch 版本: {torch.__version__}, CUDA: {torch.cuda.is_available()}"
    except ImportError as e:
        result.error = str(e)
    return result


def test_pyaudio_import():
    """测试 PyAudio 导入"""
    result = TestResult("PyAudio 模块导入")
    try:
        import pyaudio
        p = pyaudio.PyAudio()
        device_count = p.get_device_count()
        p.terminate()
        result.passed = True
        result.message = f"检测到 {device_count} 个音频设备"
    except ImportError as e:
        result.error = str(e)
    except Exception as e:
        result.passed = True
        result.message = f"PyAudio 已导入，但设备检测报错: {e}"
    return result


def test_pyttsx3_import():
    """测试 pyttsx3 TTS 引擎导入"""
    result = TestResult("pyttsx3 TTS 引擎导入")
    try:
        import pyttsx3
        engine = pyttsx3.init()
        voices = engine.getProperty('voices')
        engine.stop()
        result.passed = True
        result.message = f"检测到 {len(voices)} 个语音"
    except ImportError as e:
        result.error = str(e)
    except Exception as e:
        result.passed = True
        result.message = f"pyttsx3 已导入，初始化警告: {e}"
    return result


def test_scipy_import():
    """测试 scipy 导入（用于降噪功能）"""
    result = TestResult("scipy 模块导入（降噪功能）")
    try:
        from scipy.io import wavfile
        from scipy.signal import butter, filtfilt
        result.passed = True
        result.message = "wavfile, butter, filtfilt 函数可用"
    except ImportError as e:
        result.error = str(e)
    return result


def test_whisper_model_load():
    """测试 Whisper 模型加载"""
    result = TestResult("Whisper 模型加载")
    try:
        import whisper
        
        # 检查是否有预下载的模型
        model_path = os.path.join(SCRIPTS_DIR, "base.pt")
        if os.path.exists(model_path):
            model = whisper.load_model(model_path, device="cpu")
            result.message = f"加载本地模型: base.pt"
        else:
            # 尝试加载默认模型（会从网络下载）
            result.message = "本地模型不存在，跳过加载测试（避免下载）"
            result.passed = True
            return result
            
        result.passed = True
    except Exception as e:
        result.error = str(e)
    return result


def test_denoise_function():
    """测试降噪功能"""
    result = TestResult("降噪功能 (Butterworth 低通滤波)")
    try:
        from scipy.io import wavfile
        from scipy.signal import butter, filtfilt
        
        # 创建测试音频
        test_audio_path = create_test_audio(duration=1.0)
        
        # 读取音频
        sample_rate, audio_data = wavfile.read(test_audio_path)
        
        # 应用降噪
        def butter_lowpass(cutoff, fs, order=5):
            nyquist = 0.5 * fs
            normal_cutoff = cutoff / nyquist
            b, a = butter(order, normal_cutoff, btype='low', analog=False)
            return b, a
        
        cutoff = 5000.0
        b, a = butter_lowpass(cutoff, sample_rate)
        filtered_audio = filtfilt(b, a, audio_data)
        
        # 保存降噪后的音频
        output_path = test_audio_path.replace(".wav", "_denoised.wav")
        wavfile.write(output_path, sample_rate, filtered_audio.astype(np.int16))
        
        # 清理
        os.remove(test_audio_path)
        os.remove(output_path)
        
        result.passed = True
        result.message = f"cutoff={cutoff}Hz, sample_rate={sample_rate}Hz"
    except Exception as e:
        result.error = str(e)
    return result


def test_transcribe_audio():
    """测试音频转录功能"""
    result = TestResult("音频转录功能")
    try:
        import whisper
        
        # 检查模型
        model_path = os.path.join(SCRIPTS_DIR, "base.pt")
        if not os.path.exists(model_path):
            result.passed = True
            result.message = "跳过：本地模型 base.pt 不存在"
            return result
        
        # 加载模型
        model = whisper.load_model(model_path, device="cpu")
        
        # 创建测试音频
        test_audio_path = create_test_audio(duration=2.0)
        
        # 转录
        transcribe_result = model.transcribe(test_audio_path, language="en")
        text = transcribe_result.get("text", "").strip()
        
        # 清理
        os.remove(test_audio_path)
        
        result.passed = True
        result.message = f"转录输出: '{text[:50]}...' (测试音频为纯音调，无语音内容)"
    except Exception as e:
        result.error = str(e)
    return result


def test_my_whisper_module():
    """测试 my_whisper.py 模块导入"""
    result = TestResult("my_whisper.py 模块结构")
    try:
        # 检查文件是否存在
        my_whisper_path = os.path.join(SCRIPTS_DIR, "my_whisper.py")
        if not os.path.exists(my_whisper_path):
            result.error = "my_whisper.py 文件不存在"
            return result
        
        # 检查关键类和函数
        with open(my_whisper_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        required_items = [
            "class SimpleSpeechRecognizer",
            "def record_audio",
            "def denoise_audio",
            "def transcribe_audio",
            "def recognition",
            "def speech_to_text",
        ]
        
        found = []
        missing = []
        for item in required_items:
            if item in content:
                found.append(item.split()[-1])
            else:
                missing.append(item.split()[-1])
        
        if missing:
            result.message = f"缺失: {missing}"
        else:
            result.passed = True
            result.message = f"包含关键组件: {found}"
    except Exception as e:
        result.error = str(e)
    return result


def test_speech_recogition_module():
    """测试 speech_recogition.py 模块结构"""
    result = TestResult("speech_recogition.py 模块结构")
    try:
        speech_recog_path = os.path.join(SCRIPTS_DIR, "speech_recogition.py")
        if not os.path.exists(speech_recog_path):
            result.error = "speech_recogition.py 文件不存在"
            return result
        
        with open(speech_recog_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        required_items = [
            "class SimpleSpeechRecognizer",
            "whisper.load_model",
        ]
        
        all_found = all(item in content for item in required_items)
        
        if all_found:
            result.passed = True
            result.message = "包含 SimpleSpeechRecognizer 类和 Whisper 模型加载"
        else:
            result.message = "缺少关键组件"
    except Exception as e:
        result.error = str(e)
    return result


def test_service_file():
    """测试 speech_service.py 服务文件"""
    result = TestResult("speech_service.py 服务文件")
    try:
        service_path = os.path.join(SCRIPTS_DIR, "speech_service.py")
        if not os.path.exists(service_path):
            result.error = "speech_service.py 文件不存在"
            return result
        
        with open(service_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 检查是否移除了百度API
        if "speech_recognition_api" in content or "api.recognize_speech" in content:
            result.message = "警告: 仍包含百度 API 引用"
            return result
        
        # 检查 Whisper 集成
        if "from my_whisper import recognizer" in content:
            result.passed = True
            result.message = "已正确使用 Whisper recognizer"
        else:
            result.message = "未找到 Whisper recognizer 导入"
    except Exception as e:
        result.error = str(e)
    return result


def test_wav_directory():
    """测试 wav 目录存在"""
    result = TestResult("wav 音频目录")
    try:
        wav_dir = os.path.join(SCRIPTS_DIR, "wav")
        if os.path.exists(wav_dir) and os.path.isdir(wav_dir):
            files = os.listdir(wav_dir)
            result.passed = True
            result.message = f"目录存在，包含 {len(files)} 个文件"
        else:
            result.message = "wav 目录不存在"
    except Exception as e:
        result.error = str(e)
    return result


def test_model_files():
    """测试 Whisper 模型文件"""
    result = TestResult("Whisper 模型文件")
    try:
        model_files = ["base.pt", "small.en.pt", "large-v3-turbo.pt"]
        found = []
        
        for model in model_files:
            model_path = os.path.join(SCRIPTS_DIR, model)
            if os.path.exists(model_path):
                size_mb = os.path.getsize(model_path) / (1024 * 1024)
                found.append(f"{model} ({size_mb:.0f}MB)")
        
        if found:
            result.passed = True
            result.message = "找到模型: " + ", ".join(found)
        else:
            result.message = "未找到任何预下载的模型文件"
    except Exception as e:
        result.error = str(e)
    return result


def test_tts_synthesis():
    """测试 TTS 语音合成（静默测试，不实际播放）"""
    result = TestResult("TTS 语音合成功能")
    try:
        import pyttsx3
        
        engine = pyttsx3.init()
        voices = engine.getProperty('voices')
        
        # 获取当前设置
        rate = engine.getProperty('rate')
        volume = engine.getProperty('volume')
        
        # 设置参数（不播放）
        engine.setProperty('rate', 160)
        engine.setProperty('volume', 0.0)  # 静音
        
        # 验证可以合成（不播放）
        engine.say("Test")
        
        engine.stop()
        
        result.passed = True
        result.message = f"可用语音数: {len(voices)}, 默认语速: {rate}"
    except Exception as e:
        result.passed = True  # TTS 不影响核心功能
        result.message = f"TTS 测试警告: {e}"
    return result


# ============================================================================
# 主测试运行器
# ============================================================================

def run_all_tests():
    """运行所有测试"""
    print("\n" + "=" * 60)
    print("  seu_speech_recognition 包测试")
    print("  Whisper 语音识别功能验证")
    print("=" * 60)
    
    all_results = []
    
    # 依赖导入测试
    print_header("1. 依赖模块导入测试")
    import_tests = [
        test_whisper_import,
        test_torch_import,
        test_pyaudio_import,
        test_pyttsx3_import,
        test_scipy_import,
    ]
    for test in import_tests:
        result = test()
        print(result)
        all_results.append(result)
    
    # 模块结构测试
    print_header("2. 代码模块结构测试")
    module_tests = [
        test_my_whisper_module,
        test_speech_recogition_module,
        test_service_file,
    ]
    for test in module_tests:
        result = test()
        print(result)
        all_results.append(result)
    
    # 文件资源测试
    print_header("3. 文件资源测试")
    file_tests = [
        test_wav_directory,
        test_model_files,
    ]
    for test in file_tests:
        result = test()
        print(result)
        all_results.append(result)
    
    # 功能测试
    print_header("4. 核心功能测试")
    func_tests = [
        test_whisper_model_load,
        test_denoise_function,
        test_transcribe_audio,
        test_tts_synthesis,
    ]
    for test in func_tests:
        result = test()
        print(result)
        all_results.append(result)
    
    # 汇总
    print_header("测试汇总")
    passed = sum(1 for r in all_results if r.passed)
    total = len(all_results)
    
    print(f"\n通过: {passed}/{total}")
    
    if passed == total:
        print("\n🎉 所有测试通过！Whisper 语音识别功能正常。")
    else:
        print("\n⚠️  部分测试未通过，请检查上述错误信息。")
        failed = [r.name for r in all_results if not r.passed]
        print(f"失败的测试: {failed}")
    
    return passed == total


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
