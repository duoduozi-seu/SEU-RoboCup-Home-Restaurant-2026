#!/home/duo/anaconda3/envs/gpsr_whisper310/bin/python
import os
import shlex
import subprocess
import tempfile
import threading
import time
import wave

import rospkg
import rospy

from seu_voice.srv import seu_voice, seu_voiceResponse

PIPER_IMPORT_ERROR = ""
PiperVoice = None
try:
    from piper import PiperVoice as _PiperVoice

    PiperVoice = _PiperVoice
except Exception:
    try:
        from piper.voice import PiperVoice as _PiperVoice

        PiperVoice = _PiperVoice
    except Exception as exc:
        PIPER_IMPORT_ERROR = str(exc)


ERR_SUCCESS = 0
ERR_EMPTY_TEXT = 1001
ERR_BUSY = 1002
ERR_SYNTH_FAILED = 1003
ERR_PLAY_FAILED = 1004
ERR_MODEL_LOAD_FAILED = 1005


class VoiceServiceNode:
    def __init__(self):
        self._busy_lock = threading.Lock()
        self._voice = None
        self._load_error = ""

        pkg_path = rospkg.RosPack().get_path("seu_voice")
        default_model_path = os.path.join(pkg_path, "models", "en_US-lessac-medium.onnx")
        default_config_path = os.path.join(pkg_path, "models", "en_US-lessac-medium.onnx.json")

        self._service_name = rospy.get_param("~service_name", "voice_service")
        self._model_path = rospy.get_param("~model_path", default_model_path)
        self._config_path = rospy.get_param("~config_path", default_config_path)
        self._player_cmd = rospy.get_param("~player_cmd", "aplay -q")
        self._tmp_dir = rospy.get_param("~tmp_dir", "/tmp/seu_voice")

        self._player_tokens = shlex.split(self._player_cmd)
        if not self._player_tokens:
            raise ValueError("~player_cmd must not be empty.")

        os.makedirs(self._tmp_dir, exist_ok=True)
        self._load_voice_once()

        self._service = rospy.Service(self._service_name, seu_voice, self._handle_request)
        rospy.loginfo("seu_voice is ready on service: %s", self._service.resolved_name)

    def _load_voice_once(self):
        if PiperVoice is None:
            self._load_error = "piper import failed: {}".format(PIPER_IMPORT_ERROR)
            rospy.logerr(self._load_error)
            return

        if not os.path.exists(self._model_path):
            self._load_error = "model file not found: {}".format(self._model_path)
            rospy.logerr(self._load_error)
            return

        if not os.path.exists(self._config_path):
            self._load_error = "config file not found: {}".format(self._config_path)
            rospy.logerr(self._load_error)
            return

        try:
            self._voice = PiperVoice.load(self._model_path, config_path=self._config_path)
            rospy.loginfo("Piper model loaded: %s", self._model_path)
        except TypeError:
            # Backward compatibility with older piper APIs.
            try:
                self._voice = PiperVoice.load(self._model_path, self._config_path)
                rospy.loginfo("Piper model loaded: %s", self._model_path)
            except Exception as exc:
                self._load_error = "failed to load model/config: {}".format(exc)
                rospy.logerr(self._load_error)
        except Exception as exc:
            self._load_error = "failed to load model/config: {}".format(exc)
            rospy.logerr(self._load_error)

    @staticmethod
    def _build_response(state, errorcode, synth_t, play_t, total_t, errormsg):
        return seu_voiceResponse(
            state=state,
            errorcode=errorcode,
            synth_time_sec=float(synth_t),
            play_time_sec=float(play_t),
            total_time_sec=float(total_t),
            errormsg=errormsg,
        )

    def _synthesize_to_wav(self, text, wav_path):
        with wave.open(wav_path, "wb") as wav_file:
            if hasattr(self._voice, "synthesize_wav"):
                self._voice.synthesize_wav(text, wav_file)
            elif hasattr(self._voice, "synthesize"):
                self._voice.synthesize(text, wav_file)
            else:
                raise RuntimeError("Unsupported PiperVoice API, no synthesize method found.")

    def _handle_request(self, req):
        request_start = time.perf_counter()
        text = req.text.strip()

        if not text:
            total = time.perf_counter() - request_start
            return self._build_response(False, ERR_EMPTY_TEXT, 0.0, 0.0, total, "empty text")

        if self._voice is None:
            total = time.perf_counter() - request_start
            msg = self._load_error or "model/config not loaded"
            return self._build_response(False, ERR_MODEL_LOAD_FAILED, 0.0, 0.0, total, msg)

        if not self._busy_lock.acquire(blocking=False):
            total = time.perf_counter() - request_start
            return self._build_response(False, ERR_BUSY, 0.0, 0.0, total, "voice service is busy")

        wav_path = ""
        synth_time = 0.0
        play_time = 0.0

        try:
            try:
                with tempfile.NamedTemporaryFile(
                    mode="wb", suffix=".wav", delete=False, dir=self._tmp_dir
                ) as temp_file:
                    wav_path = temp_file.name

                synth_start = time.perf_counter()
                try:
                    self._synthesize_to_wav(text, wav_path)
                except Exception as exc:
                    total = time.perf_counter() - request_start
                    return self._build_response(
                        False,
                        ERR_SYNTH_FAILED,
                        0.0,
                        0.0,
                        total,
                        "synthesis failed: {}".format(exc),
                    )
                synth_time = time.perf_counter() - synth_start

                play_start = time.perf_counter()
                try:
                    cmd = list(self._player_tokens) + [wav_path]
                    proc = subprocess.run(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.PIPE,
                        text=True,
                        check=False,
                    )
                except Exception as exc:
                    total = time.perf_counter() - request_start
                    return self._build_response(
                        False,
                        ERR_PLAY_FAILED,
                        synth_time,
                        0.0,
                        total,
                        "player execution failed: {}".format(exc),
                    )

                play_time = time.perf_counter() - play_start
                if proc.returncode != 0:
                    total = time.perf_counter() - request_start
                    stderr = (proc.stderr or "").strip()
                    if not stderr:
                        stderr = "unknown playback error"
                    return self._build_response(
                        False,
                        ERR_PLAY_FAILED,
                        synth_time,
                        play_time,
                        total,
                        "playback failed: {}".format(stderr),
                    )

                total = time.perf_counter() - request_start
                return self._build_response(True, ERR_SUCCESS, synth_time, play_time, total, "")
            except Exception as exc:
                total = time.perf_counter() - request_start
                return self._build_response(
                    False,
                    ERR_SYNTH_FAILED,
                    0.0,
                    0.0,
                    total,
                    "voice request failed: {}".format(exc),
                )
        finally:
            if wav_path and os.path.exists(wav_path):
                try:
                    os.remove(wav_path)
                except OSError as exc:
                    rospy.logwarn("failed to remove temp wav [%s]: %s", wav_path, exc)
            self._busy_lock.release()


def main():
    rospy.init_node("seu_voice_service")
    VoiceServiceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
