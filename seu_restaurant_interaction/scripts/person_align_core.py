#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题桌边微调与 yes/no 解析核心工具。"""

import re


YES_TOKENS = (
    "yes",
    "yeah",
    "yep",
    "correct",
    "right",
    "okay",
    "ok",
    "thats right",
    "that's right",
)

NO_TOKENS = (
    "no",
    "nope",
    "wrong",
    "incorrect",
    "not right",
)


def _normalize_text(text: str) -> str:
    """将自然语言识别结果归一化，便于做包含匹配。"""
    text = (text or "").strip().lower()
    text = text.replace("’", "'")
    text = re.sub(r"[^a-z0-9'\s]+", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def classify_confirmation_text(text: str) -> str:
    """把确认语句分类成 yes / no / unknown。

    这里故意不做复杂 NLP，只做足够透明、可读、可维护的词表匹配。
    """
    normalized = _normalize_text(text)
    if not normalized:
        return "unknown"

    for token in NO_TOKENS:
        if re.search(r"(?:^|\s)" + re.escape(token) + r"(?:$|\s)", normalized):
            return "no"

    for token in YES_TOKENS:
        if re.search(r"(?:^|\s)" + re.escape(token) + r"(?:$|\s)", normalized):
            return "yes"

    return "unknown"


def select_nearest_person_within_range(candidates, max_distance_m: float):
    """在候选人列表中选出距离阈值内最近的那个人。

    Args:
        candidates: 每个候选至少包含 ``depth_m`` 字段。
        max_distance_m: 超过该距离的候选一律视为干扰人并过滤。
    """
    valid_candidates = []
    for candidate in candidates:
        depth_m = float(candidate.get("depth_m", 0.0))
        if depth_m <= 0.0:
            continue
        if depth_m > max_distance_m:
            continue
        valid_candidates.append(candidate)

    if not valid_candidates:
        return None

    return min(valid_candidates, key=lambda item: float(item["depth_m"]))
