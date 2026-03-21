#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gemini HTTP API 的最小封装。

说明：
1. 这个文件只保留最基础的 HTTP 调用、URL 拼接和文本提取；
2. 业务相关的 prompt、few-shot、输出格式约束都放在 ROS 主业务文件里；
3. API key、网关地址、模型调用方式直接照搬现有 `seu_llm` 的核心配置。
"""

import requests


# Gemini API 基础配置。
# 这里直接复用现有 `seu_llm` 中已经验证过的网关和 key。
GEMINI_API_KEY = "sk-ZzVTvgKk64GDcEFq0vf2RJzbReb6orUR9mHB0ufhwR8F2UPE"
GEMINI_BASE_URL = "https://api.viviai.cc"
GEMINI_API_VERSION = "v1beta"
HTTP_TIMEOUT_SEC = 60


class GeminiApiCore:
    """Gemini 接口最小调用封装。"""

    def __init__(self, api_key=GEMINI_API_KEY, base_url=GEMINI_BASE_URL,
                 api_version=GEMINI_API_VERSION, timeout_sec=HTTP_TIMEOUT_SEC):
        """保存基础配置，供后续每次请求重复使用。"""
        self.api_key = api_key
        self.base_url = base_url
        self.api_version = api_version
        self.timeout_sec = timeout_sec

    def build_generate_content_url(self, model_name):
        """拼接 Gemini generateContent 接口地址。"""
        return "{}/{}/models/{}:generateContent".format(
            self.base_url.rstrip("/"),
            self.api_version,
            model_name,
        )

    def build_headers(self):
        """构造统一请求头。"""
        return {
            "Authorization": "Bearer {}".format(self.api_key),
            "Content-Type": "application/json",
        }

    def build_payload(self, prompt_text):
        """把一段 prompt 包成 Gemini 所需的请求体。"""
        return {
            "contents": [
                {
                    "role": "user",
                    "parts": [
                        {
                            "text": prompt_text,
                        }
                    ],
                }
            ]
        }

    def extract_text_from_response_json(self, response_json):
        """从 Gemini 标准返回 JSON 中提取文本。"""
        if not isinstance(response_json, dict):
            return ""

        candidates = response_json.get("candidates", [])
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

        text_list = []
        for part in parts:
            if not isinstance(part, dict):
                continue
            part_text = part.get("text", "")
            if not isinstance(part_text, str):
                continue
            clean_text = part_text.strip()
            if not clean_text:
                continue
            text_list.append(clean_text)

        return "\n".join(text_list).strip()

    def generate_content(self, prompt_text, model_name):
        """发送一次 Gemini 请求，并返回统一结果结构。"""
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
                url=self.build_generate_content_url(model_name),
                headers=self.build_headers(),
                json=self.build_payload(prompt_text),
                timeout=self.timeout_sec,
            )
        except requests.RequestException as exc:
            result["error"] = "RequestException: {}".format(exc)
            return result

        result["status_code"] = response.status_code
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

        if response.status_code == 200 and result["text"]:
            result["ok"] = True
        elif response.status_code != 200:
            result["error"] = "HTTP {}: {}".format(response.status_code, response.text)
        else:
            result["error"] = "Gemini response text is empty after parsing."

        return result
