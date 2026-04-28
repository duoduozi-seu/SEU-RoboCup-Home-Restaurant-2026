#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal Qwen HTTP API wrapper for the keyword extractor.

This module keeps the same public shape as gemini_api_core.py:
call generate_content(prompt_text, model_name) and get a normalized result dict.
The implementation talks to Alibaba Cloud DashScope's OpenAI-compatible
chat/completions endpoint directly with requests, so the ROS package does not
need an extra openai SDK dependency.
"""

import json
import os

import requests


QWEN_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
QWEN_API_KEY_ENV = "DASHSCOPE_API_KEY"
# Put your DashScope API key here if you want it hard-coded, for example:
# QWEN_API_KEY = "sk-xxxxxxxx"
QWEN_API_KEY = "sk-75cad65b35904a9abf38f3f7a0612485"
QWEN_MODEL = "qwen3.6-max-preview"
HTTP_TIMEOUT_SEC = 60
QWEN_ENABLE_THINKING = False
QWEN_STREAM = True

ERROR_CODE_MISSING_API_KEY = "MISSING_API_KEY"
ERROR_CODE_NETWORK_TIMEOUT = "NETWORK_TIMEOUT"
ERROR_CODE_NETWORK_CONNECTION = "NETWORK_CONNECTION_ERROR"
ERROR_CODE_REQUEST_EXCEPTION = "REQUEST_EXCEPTION"
ERROR_CODE_EMPTY_RESPONSE = "EMPTY_RESPONSE"


class QwenApiCore:
    """Small wrapper around Qwen chat completions."""

    def __init__(self, api_key=None, base_url=QWEN_BASE_URL,
                 timeout_sec=HTTP_TIMEOUT_SEC,
                 enable_thinking=QWEN_ENABLE_THINKING,
                 stream=QWEN_STREAM):
        if api_key is not None:
            self.api_key = api_key
        elif QWEN_API_KEY:
            self.api_key = QWEN_API_KEY
        else:
            self.api_key = os.getenv(QWEN_API_KEY_ENV, "")
        self.base_url = base_url
        self.timeout_sec = timeout_sec
        self.enable_thinking = enable_thinking
        self.stream = stream

    def build_chat_completions_url(self):
        """Build DashScope OpenAI-compatible chat/completions URL."""
        return "{}/chat/completions".format(self.base_url.rstrip("/"))

    def build_headers(self):
        """Build request headers."""
        return {
            "Authorization": "Bearer {}".format(self.api_key),
            "Content-Type": "application/json",
        }

    def build_messages(self, prompt_text):
        """Wrap one prompt as OpenAI-compatible chat messages."""
        return [
            {
                "role": "user",
                "content": str(prompt_text),
            }
        ]

    def build_payload(self, prompt_text, model_name):
        """Build OpenAI-compatible request body for Qwen."""
        payload = {
            "model": model_name,
            "messages": self.build_messages(prompt_text),
            "stream": bool(self.stream),
        }
        if self.enable_thinking is not None:
            payload["enable_thinking"] = bool(self.enable_thinking)
        return payload

    def extract_text_from_response_json(self, response_json):
        """Extract assistant content from a non-streaming chat response."""
        if not isinstance(response_json, dict):
            return ""

        choices = response_json.get("choices", [])
        if not isinstance(choices, list) or not choices:
            return ""

        choice = choices[0]
        if not isinstance(choice, dict):
            return ""

        message = choice.get("message", {})
        if not isinstance(message, dict):
            return ""

        content = message.get("content", "")
        if not isinstance(content, str):
            return ""
        return content.strip()

    def _extract_stream_chunk_text(self, chunk_json):
        if not isinstance(chunk_json, dict):
            return "", ""

        choices = chunk_json.get("choices", [])
        if not isinstance(choices, list) or not choices:
            return "", ""

        choice = choices[0]
        if not isinstance(choice, dict):
            return "", ""

        delta = choice.get("delta", {})
        if not isinstance(delta, dict):
            return "", ""

        reasoning_content = delta.get("reasoning_content", "")
        if not isinstance(reasoning_content, str):
            reasoning_content = ""

        content = delta.get("content", "")
        if not isinstance(content, str):
            content = ""

        return reasoning_content, content

    def _read_stream_response(self, response, result):
        raw_line_list = []
        chunk_json_list = []
        reasoning_text_list = []
        content_text_list = []

        for raw_line in response.iter_lines(decode_unicode=True):
            if isinstance(raw_line, bytes):
                raw_line = raw_line.decode("utf-8", "replace")
            if not isinstance(raw_line, str):
                continue
            if not raw_line:
                continue

            raw_line_list.append(raw_line)
            if not raw_line.startswith("data:"):
                continue

            data_text = raw_line[len("data:"):].strip()
            if data_text == "[DONE]":
                break

            try:
                chunk_json = json.loads(data_text)
            except ValueError:
                continue

            chunk_json_list.append(chunk_json)
            reasoning_text, content_text = self._extract_stream_chunk_text(chunk_json)
            if reasoning_text:
                reasoning_text_list.append(reasoning_text)
            if content_text:
                content_text_list.append(content_text)

        result["raw_text"] = "\n".join(raw_line_list)
        result["data"] = chunk_json_list
        result["text"] = "".join(content_text_list).strip()
        result["reasoning_text"] = "".join(reasoning_text_list).strip()

    def _read_non_stream_response(self, response, result):
        result["raw_text"] = response.text

        response_json = None
        try:
            response_json = response.json()
            result["data"] = response_json
        except ValueError:
            response_json = None

        if response_json is not None:
            result["text"] = self.extract_text_from_response_json(response_json)

        if not result["text"]:
            result["text"] = (response.text or "").strip()

    def generate_content(self, prompt_text, model_name):
        """Send one Qwen request and return a normalized result dict."""
        result = {
            "ok": False,
            "status_code": None,
            "error_code": "",
            "text": "",
            "raw_text": "",
            "data": None,
            "error": "",
        }

        if not self.api_key:
            result["error_code"] = ERROR_CODE_MISSING_API_KEY
            result["error"] = "{} is not configured.".format(QWEN_API_KEY_ENV)
            return result

        try:
            response = requests.post(
                url=self.build_chat_completions_url(),
                headers=self.build_headers(),
                json=self.build_payload(prompt_text, model_name),
                timeout=self.timeout_sec,
                stream=bool(self.stream),
            )
            result["status_code"] = response.status_code

            if bool(self.stream) and response.status_code == 200:
                self._read_stream_response(response, result)
            else:
                self._read_non_stream_response(response, result)
        except requests.Timeout as exc:
            result["error_code"] = ERROR_CODE_NETWORK_TIMEOUT
            result["error"] = "RequestException: {}".format(exc)
            return result
        except requests.ConnectionError as exc:
            result["error_code"] = ERROR_CODE_NETWORK_CONNECTION
            result["error"] = "RequestException: {}".format(exc)
            return result
        except requests.RequestException as exc:
            result["error_code"] = ERROR_CODE_REQUEST_EXCEPTION
            result["error"] = "RequestException: {}".format(exc)
            return result

        if result["status_code"] == 200 and result["text"]:
            result["ok"] = True
        elif result["status_code"] != 200:
            result["error_code"] = "HTTP_{}".format(result["status_code"])
            result["error"] = "HTTP {}: {}".format(result["status_code"], result["raw_text"])
        else:
            result["error_code"] = ERROR_CODE_EMPTY_RESPONSE
            result["error"] = "Qwen response text is empty after parsing."

        return result
