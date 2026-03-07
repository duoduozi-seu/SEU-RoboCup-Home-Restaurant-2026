#!/home/duo/anaconda3/envs/gpsr_whisper310/bin/python

import os
import sys
import time
import wave
from contextlib import contextmanager

import pyaudio
import rospy
import torch
from faster_whisper import WhisperModel

import config
from seu_speech_recognition.srv import seu_speech_recognitionResponse

AUDIO_OUTPUT_PATH = config.AUDIO_OUTPUT_PATH
WAV_PATH = config.AUDIO_OUTPUT_PATH


@contextmanager
def ignore_stderr():
    """Hide noisy ALSA/PortAudio stderr output while opening microphone."""
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


def resolve_model_name(model_name):
    """Allow legacy .pt config and map to faster-whisper model names."""
    if not model_name:
        return "large-v3-turbo"

    if os.path.isfile(model_name) and model_name.endswith(".pt"):
        basename = os.path.basename(model_name).lower()
        mapping = {
            "tiny.pt": "tiny",
            "tiny.en.pt": "tiny.en",
            "base.pt": "base",
            "base.en.pt": "base.en",
            "small.pt": "small",
            "small.en.pt": "small.en",
            "medium.pt": "medium",
            "medium.en.pt": "medium.en",
            "large-v3.pt": "large-v3",
            "large-v3-turbo.pt": "large-v3-turbo",
        }
        resolved = mapping.get(basename, "large-v3-turbo")
        print(
            "[seu_speech_recognition] .pt path detected, "
            "use faster-whisper model: {}".format(resolved)
        )
        return resolved

    return model_name


class FasterWhisperRecognizer:
    def __init__(self, model_name="large-v3-turbo", language="en"):
        self.language = language
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.compute_type = "float16" if self.device == "cuda" else "int8"
        self.model_name = resolve_model_name(model_name)

        print("Loading faster-whisper model: {}".format(self.model_name))
        print("device={}, compute_type={}".format(self.device, self.compute_type))
        self.model = WhisperModel(
            self.model_name,
            device=self.device,
            compute_type=self.compute_type,
        )

    def record_audio(self, output_audio_path=AUDIO_OUTPUT_PATH, max_duration=10):
        with ignore_stderr():
            p = pyaudio.PyAudio()

        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024,
        )

        frames = []
        for _ in range(int(16000 / 1024 * max_duration)):
            frames.append(stream.read(1024))

        stream.stop_stream()
        stream.close()
        p.terminate()

        with wave.open(output_audio_path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b"".join(frames))

        return output_audio_path

    def transcribe_audio(self, audio_path):
        start = time.perf_counter()
        segments, info = self.model.transcribe(
            audio_path,
            language=self.language,
            beam_size=1,
        )
        text = "".join(segment.text for segment in segments).strip()
        elapsed = time.perf_counter() - start
        print(
            "faster-whisper done: {:.3f}s, language={}, prob={:.3f}".format(
                elapsed,
                getattr(info, "language", "unknown"),
                float(getattr(info, "language_probability", 0.0)),
            )
        )
        return text

    def recognition(self, req=None, path=WAV_PATH):
        try:
            recorded_path = self.record_audio(output_audio_path=path)
            word = self.transcribe_audio(recorded_path).lower()
            if word:
                return seu_speech_recognitionResponse(True, 0, word, "success")
            return seu_speech_recognitionResponse(False, 1, "", "empty")
        except Exception as exc:
            rospy.logerr("faster-whisper recognition failed: %s", exc)
            return seu_speech_recognitionResponse(False, -1, "", str(exc))


recognizer = FasterWhisperRecognizer(model_name=config.MODEL_PATH, language="en")
