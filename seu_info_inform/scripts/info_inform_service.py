#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import re

import requests
import rospy
from seu_info_inform.srv import seu_info_inform, seu_info_informResponse
from seu_speech_recognition.srv import (
    seu_speech_recognition,
    seu_speech_recognitionRequest,
)
from seu_voice.srv import seu_voice, seu_voiceRequest

from info_inform_prompt_data import (
    GEMINI_API_KEY,
    GEMINI_API_VERSION,
    GEMINI_BASE_URL,
    GEMINI_MODEL_INFO_INFORM,
    HTTP_TIMEOUT_SEC,
    INFO_INFORM_ANSWER_OPENING,
    INFO_INFORM_OUTPUT_RETRY_HINT,
    MAX_RETRY_TIMES,
    build_answer_prompt,
    build_report_prompt,
)


GREEN_TAG = "\033[32m[info_inform]\033[0m"
RED_TAG = "\033[31m[info_inform]\033[0m"
SPEECH_PATTERN = re.compile(r"Speech:\s*(.*)", re.IGNORECASE | re.DOTALL)
VOICE_SERVICE_NAME = "voice_service"
voice_client = None


def info_loginfo(msg, *args):
    rospy.loginfo(f"{GREEN_TAG} {msg}", *args)


def info_logerr(msg, *args):
    rospy.logerr(f"{RED_TAG} {msg}", *args)


def speak_text(text):
    """Speak text through seu_voice."""
    global voice_client

    text = str(text).strip()
    if not text:
        return

    if voice_client is None:
        rospy.wait_for_service(VOICE_SERVICE_NAME)
        voice_client = rospy.ServiceProxy(VOICE_SERVICE_NAME, seu_voice)
        info_loginfo("%s available", VOICE_SERVICE_NAME)

    response = voice_client.call(seu_voiceRequest(text=text))
    if not response.state:
        info_logerr("Voice service failed: %s", response.errormsg)


def srv_speech_recognition(task="none"):
    """Call the speech recognition service."""
    srv_name = "speech_recognition_service"
    client = rospy.ServiceProxy(srv_name, seu_speech_recognition)
    rospy.wait_for_service(srv_name)
    info_loginfo("%s available", srv_name)
    try:
        request = seu_speech_recognitionRequest()
        request.task = task
        request.enable = False
        response = client.call(request)
        if response.state:
            return response.word
        info_logerr("Speech recognition failed: %s", response.errormsg)
        return -1
    except rospy.ServiceException as exc:
        info_logerr("Speech recognition service call failed: %s", exc)
        return -1


def build_generate_content_url(model):
    return (
        f"{GEMINI_BASE_URL.rstrip('/')}/{GEMINI_API_VERSION}/models/"
        f"{model}:generateContent"
    )


def build_headers():
    return {
        "Authorization": f"Bearer {GEMINI_API_KEY}",
        "Content-Type": "application/json",
    }


def build_payload(user_text):
    return {
        "contents": [
            {
                "role": "user",
                "parts": [{"text": user_text}],
            }
        ]
    }


def extract_text_from_response_json(data):
    """Extract text from Gemini JSON response."""
    if not isinstance(data, dict):
        return ""

    candidates = data.get("candidates", [])
    if not isinstance(candidates, list) or not candidates:
        return ""

    candidate = candidates[0]
    if not isinstance(candidate, dict):
        return ""

    content = candidate.get("content", {})
    if not isinstance(content, dict):
        return ""

    parts = content.get("parts", [])
    if not isinstance(parts, list):
        return ""

    texts = []
    for part in parts:
        if not isinstance(part, dict):
            continue
        part_text = part.get("text")
        if not isinstance(part_text, str):
            continue
        cleaned = part_text.strip()
        if not cleaned:
            continue
        if texts and texts[-1] == cleaned:
            continue
        texts.append(cleaned)

    return "\n".join(texts).strip()


def call_gemini_generate_content(prompt, model):
    """Call Gemini and return a normalized result dict."""
    result = {
        "ok": False,
        "status_code": None,
        "text": "",
        "raw_text": "",
        "data": None,
        "error": "",
    }

    try:
        response = requests.post(
            url=build_generate_content_url(model),
            headers=build_headers(),
            json=build_payload(prompt),
            timeout=HTTP_TIMEOUT_SEC,
        )
    except requests.RequestException as exc:
        result["error"] = f"RequestException: {exc}"
        return result

    result["status_code"] = response.status_code
    result["raw_text"] = response.text

    try:
        result["data"] = response.json()
    except ValueError:
        result["data"] = None

    result["text"] = extract_text_from_response_json(result["data"])
    if not result["text"]:
        result["text"] = (response.text or "").strip()

    if response.status_code == 200 and result["text"]:
        result["ok"] = True
        return result

    if response.status_code != 200:
        result["error"] = f"HTTP {response.status_code}: {response.text}"
    elif not result["text"]:
        result["error"] = "Response text is empty after parsing."
    return result


def should_retry_same_prompt(api_result):
    """Retry unchanged prompt on transport or 5xx failure."""
    status_code = api_result.get("status_code")
    error = str(api_result.get("error", ""))
    if status_code is None and error.startswith("RequestException:"):
        return True
    if isinstance(status_code, int) and status_code >= 500:
        return True
    return False


def extract_speech_text(response_text):
    """Parse `Speech: ...` from the model output."""
    if not response_text:
        return ""
    match = SPEECH_PATTERN.search(response_text)
    if not match:
        return ""
    speech = match.group(1).strip()
    speech = speech.replace("\n", " ").strip()
    return speech


class InfoInformService:
    """Single-entry LLM-driven info_inform service."""

    def __init__(self):
        self.model = GEMINI_MODEL_INFO_INFORM

    def _generate_speech(self, prompt):
        """Generate one `Speech:` line from Gemini with format retries."""
        current_prompt = prompt
        for attempt in range(MAX_RETRY_TIMES):
            api_result = call_gemini_generate_content(current_prompt, self.model)
            info_loginfo("Gemini attempt %d/%d", attempt + 1, MAX_RETRY_TIMES)

            if not api_result["ok"]:
                info_logerr("Gemini request issue: %s", api_result["error"])
                if should_retry_same_prompt(api_result):
                    continue

            speech = extract_speech_text(api_result["text"])
            if speech:
                info_loginfo("Parsed speech: %s", speech)
                return speech

            info_logerr("Failed to parse `Speech:` output. Retrying...")
            current_prompt = (
                f"[Retry hint]\n{INFO_INFORM_OUTPUT_RETRY_HINT}\n\n{prompt}"
            )

        return ""

    def _answer_flow(self, req):
        """Special flow for `answer`: ask question first, then answer via LLM."""
        speak_text(INFO_INFORM_ANSWER_OPENING)
        user_question = srv_speech_recognition("answer")
        if user_question == -1 or not str(user_question).strip():
            return seu_info_informResponse(False, "failed to hear the question")

        prompt = build_answer_prompt(
            ori_command=req.ori_command,
            result_text=req.result_text,
            user_question=str(user_question).strip(),
        )
        speech = self._generate_speech(prompt)
        if not speech:
            return seu_info_informResponse(False, "failed to generate answer speech")

        speak_text(speech)
        return seu_info_informResponse(True, "success")

    def _report_flow(self, req):
        """General flow for all non-answer info kinds."""
        prompt = build_report_prompt(
            info_kind=req.info_kind,
            ori_command=req.ori_command,
            result_text=req.result_text,
        )
        speech = self._generate_speech(prompt)
        if not speech:
            return seu_info_informResponse(False, "failed to generate report speech")

        speak_text(speech)
        return seu_info_informResponse(True, "success")

    def callback(self, req):
        info_loginfo(
            "Received info_inform request: info_kind=%s, result_text=%s",
            req.info_kind,
            req.result_text,
        )
        if req.info_kind == "answer":
            return self._answer_flow(req)
        return self._report_flow(req)


if __name__ == "__main__":
    rospy.init_node("info_inform_node")
    info_inform = InfoInformService()
    server = rospy.Service(
        "info_inform_service",
        seu_info_inform,
        info_inform.callback,
    )
    info_loginfo("info inform service ready")
    rospy.spin()
