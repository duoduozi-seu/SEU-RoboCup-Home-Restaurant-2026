#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
Gemini HTTP API 的轻量封装层。

职责：
1. 统一拼接请求 URL、Header、Payload。
2. 统一执行 HTTP 请求。
3. 统一解析 Gemini 返回文本，并提供 JSON 解析失败时的兜底文本。
"""

import requests

from configs import (
    ENABLE_RAW_TEXT_FALLBACK,
    GEMINI_API_KEY,
    GEMINI_API_VERSION,
    GEMINI_BASE_URL,
    HTTP_TIMEOUT_SEC,
)


def build_generate_content_url(model):
    """构造 generateContent 的完整 URL。"""
    return f"{GEMINI_BASE_URL.rstrip('/')}/{GEMINI_API_VERSION}/models/{model}:generateContent"


def build_headers():
    """构造统一请求头。"""
    return {
        "Authorization": f"Bearer {GEMINI_API_KEY}",
        "Content-Type": "application/json",
    }


def build_payload(user_text):
    """构造 Gemini 文本输入请求体。"""
    return {
        "contents": [
            {
                "role": "user",
                "parts": [
                    {
                        "text": user_text,
                    }
                ],
            }
        ]
    }


def extract_text_from_response_json(data):
    """
    从 Gemini 标准 JSON 返回中提取文本。

    常见结构：
    data["candidates"][0]["content"]["parts"][0]["text"]
    """
    if not isinstance(data, dict):
        return ""

    candidates = data.get("candidates", [])
    if not isinstance(candidates, list) or not candidates:
        return ""

    # 仅使用第一个候选，避免多候选拼接导致重复答案
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
        # 去除连续重复文本片段，防止 "Answer: x" 被拼两次
        if texts and texts[-1] == cleaned:
            continue
        texts.append(cleaned)

    return "\n".join(texts).strip()


def call_gemini_generate_content(prompt, model):
    """
    调用 Gemini 接口并返回统一结构，避免业务层反复写解析逻辑。

    返回字段：
    - ok: bool，请求与文本解析是否成功
    - status_code: int 或 None
    - text: 解析后的文本（JSON优先 + 原文兜底）
    - raw_text: 原始响应文本
    - data: JSON对象或 None
    - error: 错误信息字符串
    """
    url = build_generate_content_url(model)
    headers = build_headers()
    payload = build_payload(prompt)

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
            url=url,
            headers=headers,
            json=payload,
            timeout=HTTP_TIMEOUT_SEC,
        )
    except requests.RequestException as exc:
        result["error"] = f"RequestException: {exc}"
        return result

    result["status_code"] = response.status_code
    result["raw_text"] = response.text

    data = None
    try:
        data = response.json()
        result["data"] = data
    except ValueError:
        data = None

    # 优先使用结构化 JSON 提取文本，保证兼容规范响应。
    parsed_text = extract_text_from_response_json(data) if data is not None else ""

    # 兜底：如果 JSON 结构不标准，仍然保留原始文本供上层正则继续匹配。
    if not parsed_text and ENABLE_RAW_TEXT_FALLBACK:
        parsed_text = (response.text or "").strip()

    result["text"] = parsed_text

    if response.status_code == 200 and parsed_text:
        result["ok"] = True
    else:
        if response.status_code != 200:
            result["error"] = f"HTTP {response.status_code}: {response.text}"
        elif not parsed_text:
            result["error"] = "Response text is empty after parsing."

    return result
