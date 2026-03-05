#!/home/duo/anaconda3/envs/gpsr_whisper310/bin/python
# -*- coding: UTF-8 -*-
"""
Gemini 版本的 ROS LLM 服务。

功能目标：
1. 与 baidu_llm_service.py 保持同等流程能力：
   - task == analyze：把自然语言任务解析成动作列表字符串；
   - task == answer：回答英文问题并返回文本。
2. 通过 configs.py 管理所有硬编码参数。
3. 通过 gemini_api.py 管理所有 HTTP 调用和基础解析。
"""

import ast
import json
import re

import rospy

from seu_llm.srv import seu_llm, seu_llmResponse

from configs import (
    ACTION_BASE,
    ACTION_BASE_LIST,
    ANALYZE_INSTRUCTION_PROMPT,
    ANALYZE_PATTERN,
    ANSWER_INSTRUCTION_PROMPT,
    ANSWER_PATTERN,
    ANSWER_REFERENCE,
    DEFAULT_FAILURE_ANSWER,
    DEFAULT_FAILURE_PLAN,
    GEMINI_MODEL_ANALYZE,
    GEMINI_MODEL_ANSWER,
    LLM_NODE_NAME,
    MAX_RETRY_TIMES,
    PLAN_SET_PATH,
    PROMPT_ADD_FORMAT,
    PROMPT_ADD_MANDATED_SUBTASKS,
    PROMPT_KNOWN,
    SEUHOME_LLM_NAME,
)
from gemini_api import call_gemini_generate_content

GREEN_GEMINI_TAG = "\033[32m[gemini]\033[0m"
RED_GEMINI_TAG = "\033[31m[gemini]\033[0m"


def gemini_loginfo(msg, *args):
    """Gemini 普通日志：绿色 [gemini] 前缀。"""
    rospy.loginfo(f"{GREEN_GEMINI_TAG} {msg}", *args)


def gemini_logerr(msg, *args):
    """Gemini 错误日志：红色 [gemini] 前缀。"""
    rospy.logerr(f"{RED_GEMINI_TAG} {msg}", *args)


def llm_service(req):
    """
    统一服务入口：
    - req.task == 'analyze'：执行任务规划；
    - req.task == 'answer'：执行问题回答。
    """
    if req.task == "analyze":
        return analyze(req)
    if req.task == "answer":
        return get_answer(req)

    gemini_logerr("Unsupported task type: %s", req.task)
    return seu_llmResponse(False, "failure", "Unsupported task type")


def _load_plan_examples():
    """
    读取任务规划样例（plan_set_2025.json）。
    返回 dict；读取失败时返回 None。
    """
    try:
        with open(PLAN_SET_PATH, "r") as f:
            return json.load(f)
    except Exception as exc:  # pylint: disable=broad-except
        gemini_logerr("Failed to load plan set file: %s, error: %s", PLAN_SET_PATH, exc)
        return None


def _build_analyze_base_prompt(examples):
    """
    构造“任务规划”的基础提示词骨架。
    """
    prompt = f"from actions import {ACTION_BASE}"
    prompt += f"\n\nexamples = {examples}"
    prompt += f"\n\n{ANALYZE_INSTRUCTION_PROMPT}"
    return prompt


def _build_analyze_prompt(base_prompt, task_text, extra_hint=""):
    """
    在基础提示词上拼接“场景先验 + 当前任务 + 额外提示”。
    """
    prompt = (
        f"{base_prompt}\n\n"
        f"[Known environment facts]\n{PROMPT_KNOWN}\n\n"
        f"[User task]\n{task_text}\n"
    )
    if extra_hint:
        prompt += f"\n[Retry hint]\n{extra_hint}\n"
    return prompt


def _extract_action_str(response_text):
    """
    用正则从模型输出中提取动作列表字符串，如：
    [[\"move_to\", \"kitchen\"], [\"speak\", \"hello\"]]
    """
    if not response_text:
        return ""
    match = re.search(ANALYZE_PATTERN, response_text, re.DOTALL)
    return match.group() if match else ""


def _is_valid_action_list(action_str):
    """
    校验动作列表是否可执行：
    1. 必须能被 literal_eval 解析成 list；
    2. 每个子项必须是 list/tuple；
    3. 每个动作名必须存在于 ACTION_BASE_LIST。
    """
    try:
        action_lst = ast.literal_eval(action_str)
    except (ValueError, SyntaxError):
        return False

    if not isinstance(action_lst, list):
        return False

    for action in action_lst:
        if not isinstance(action, (list, tuple)) or len(action) < 1:
            return False
        if action[0] not in ACTION_BASE_LIST:
            gemini_logerr("Found unsupported action: %s", action[0])
            return False

    return True


def analyze(req):
    """
    任务规划主流程（与 baidu_llm_service 的处理逻辑一致）：
    1. 构造规划提示词；
    2. 调用 Gemini；
    3. 正则提取动作列表；
    4. 校验动作可执行性；
    5. 失败时按规则重试。
    """
    gemini_loginfo("Start to analyze the command")
    prompt_task = req.command
    gemini_loginfo("Task: %s", prompt_task)

    examples = _load_plan_examples()
    if examples is None:
        return seu_llmResponse(False, "failure", DEFAULT_FAILURE_PLAN)

    base_prompt = _build_analyze_base_prompt(examples)
    current_prompt = _build_analyze_prompt(base_prompt, prompt_task)

    for i in range(MAX_RETRY_TIMES):
        api_result = call_gemini_generate_content(current_prompt, GEMINI_MODEL_ANALYZE)
        gemini_loginfo("Analyze attempt %d/%d", i + 1, MAX_RETRY_TIMES)

        if not api_result["ok"]:
            gemini_logerr("Gemini analyze request issue: %s", api_result["error"])

        action_str = _extract_action_str(api_result["text"])
        if action_str:
            if _is_valid_action_list(action_str):
                gemini_loginfo("Task plan successfully!")
                return seu_llmResponse(True, "success", action_str)

            gemini_logerr("Planned subtasks are not executable. Retrying...")
            current_prompt = _build_analyze_prompt(
                base_prompt,
                prompt_task,
                PROMPT_ADD_MANDATED_SUBTASKS,
            )
            continue

        gemini_logerr("Failed to extract action list. Retrying...")
        current_prompt = _build_analyze_prompt(
            base_prompt,
            prompt_task,
            PROMPT_ADD_FORMAT,
        )

    gemini_logerr("Task planning failed after %d attempts.", MAX_RETRY_TIMES)
    return seu_llmResponse(False, "failure", DEFAULT_FAILURE_PLAN)


def _build_answer_prompt(question, extra_hint=""):
    """
    构造问答提示词。
    """
    base_prompt = (
        f"{ANSWER_INSTRUCTION_PROMPT}\n\n"
        f"[Question]\n{question}\n\n"
        "[Reference examples for style only]\n"
        "Use these references only as style guidance; do not copy or repeat them.\n"
        f"{ANSWER_REFERENCE}\n"
    )
    if extra_hint:
        return f"[Retry hint]\n{extra_hint}\n\n{base_prompt}"
    return base_prompt


def _extract_answer_text(response_text):
    """
    从模型输出中提取最终答案文本。
    期望格式：Answer: xxx
    """
    if not response_text:
        return ""

    # 使用 findall 获取所有 Answer 段，优先取最后一个有效段。
    # 这样即使模型重复输出两次 "Answer: ..."，也不会把两段粘连返回。
    matches = re.findall(ANSWER_PATTERN, response_text, re.IGNORECASE | re.DOTALL)
    if not matches:
        return ""

    for segment in reversed(matches):
        cleaned = segment.strip()
        if cleaned:
            return cleaned

    return ""


def get_answer(req):
    """
    问答主流程（与 baidu_llm_service 的处理逻辑一致）：
    1. 构造问答 prompt；
    2. 调用 Gemini；
    3. 正则提取 Answer: 后面的内容；
    4. 格式不满足则重试。
    """
    gemini_loginfo("Start to answer the question")
    tts_question = req.command

    current_prompt = _build_answer_prompt(tts_question)

    for i in range(MAX_RETRY_TIMES):
        api_result = call_gemini_generate_content(current_prompt, GEMINI_MODEL_ANSWER)
        gemini_loginfo("Answer attempt %d/%d", i + 1, MAX_RETRY_TIMES)

        if not api_result["ok"]:
            gemini_logerr("Gemini answer request issue: %s", api_result["error"])

        tts_answer = _extract_answer_text(api_result["text"])
        if tts_answer:
            gemini_loginfo("Answer successfully extracted: %s", tts_answer)
            return seu_llmResponse(True, "success", tts_answer)

        gemini_logerr("Failed to parse answer format. Retrying...")
        current_prompt = _build_answer_prompt(tts_question, PROMPT_ADD_FORMAT)

    gemini_logerr("Failed to get answer after %d attempts.", MAX_RETRY_TIMES)
    return seu_llmResponse(False, "failure", DEFAULT_FAILURE_ANSWER)


if __name__ == "__main__":
    rospy.init_node(LLM_NODE_NAME)
    server = rospy.Service(SEUHOME_LLM_NAME, seu_llm, llm_service)
    gemini_loginfo("Ready to analyze command or answer question. service=%s", SEUHOME_LLM_NAME)
    rospy.spin()
