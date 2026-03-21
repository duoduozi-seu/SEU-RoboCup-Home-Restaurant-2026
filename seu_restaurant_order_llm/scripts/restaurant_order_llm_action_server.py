#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题订单物品解析 action server。

职责：
1. 接收一整句英文订单文本；
2. 调用 Gemini 做关键物品抽取；
3. 返回按说话顺序整理好的 `string[] items`；
4. 当前阶段只提供独立 action 节点，不改动状态机接线。
"""

import json
import os
import re
import sys

import actionlib
import rospy


# 这里主动把当前脚本目录加入导入路径，保证直接 rosrun scripts 文件时导入稳定。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from gemini_api_core import GeminiApiCore


# ROS 节点与 action 名称。
NODE_NAME = "restaurant_order_llm_action_server"
ACTION_NAME = "parse_restaurant_order"

# Gemini 模型与请求重试次数。
GEMINI_MODEL = "gemini-2.5-flash"
MAX_RETRY_TIMES = 3

# 规则 prompt：强约束模型只输出一个 JSON 数组。
# 这里不再给任何硬编码候选词表，也不写自定义映射关系，只做通用抽取。
ORDER_PARSE_RULE_PROMPT = """You are an item extraction module for a restaurant service robot.
Your job is to read one English customer order sentence and extract the requested items.

Strict rules:
1. Output only one JSON array of strings.
2. Do not output explanations, markdown, code fences, or extra words.
3. Keep the item order exactly the same as the customer mentioned.
4. If the quantity is clearly more than one, repeat the item in the array.
5. Keep each item as a short lower-case English item phrase.
6. Do not invent any special mapping relation. Keep the original item meaning.
7. If the speech recognition text has small mistakes, infer the most likely intended item phrase.
8. Remove quantity words and packaging helper words like bottle, bag, cup, can, pack, or piece when they are not the item itself.
9. If no valid item can be extracted, output [].

Output only one JSON array, for example: ["cola", "chips", "mineral water"]"""

# few-shot prompt：直接给模型示范输入和目标输出风格。
ORDER_PARSE_FEW_SHOT_PROMPT = """[Few-shot examples]
Customer: i want a bottle of cola and a bag of chips and a bottle of mineral water
Output: ["cola", "chips", "mineral water"]

Customer: please give me two bottles of water and one cookie
Output: ["water", "water", "cookie"]

Customer: can i have orange juice and a pack of crackers
Output: ["orange juice", "crackers"]

Customer: i would like bread, chocolate milk and apple juice
Output: ["bread", "chocolate milk", "apple juice"]"""

# 当模型第一次格式不对时，第二次会补一段更硬的提醒。
RETRY_HINT_PROMPT = """[Retry instruction]
Your previous answer did not follow the required format.
Regenerate and output only one JSON array of strings."""


def build_order_parse_prompt(order_text, retry_hint_text=""):
    """把规则、few-shot 和当前用户句子拼成一次完整 prompt。"""
    # 先放规则，保证模型一开始就知道只能输出 JSON 数组。
    prompt = ORDER_PARSE_RULE_PROMPT
    prompt += "\n\n"

    # 再给 few-shot，让模型看到我们希望的输入输出样式。
    prompt += ORDER_PARSE_FEW_SHOT_PROMPT
    prompt += "\n\n"

    # 最后放当前顾客的真实句子，避免它把示例内容混进结果里。
    prompt += "[Current customer sentence]\n"
    prompt += order_text.strip()

    # 如果前一次输出格式不对，就在这里追加 retry 提示。
    if retry_hint_text:
        prompt += "\n\n"
        prompt += retry_hint_text.strip()

    return prompt


def extract_first_json_array_text(model_text):
    """从模型原始文本中截出第一个 JSON 数组。

    这里故意不用复杂语法，只做最直接的方括号定位：
    1. 找到第一个 `[`；
    2. 再找到最后一个 `]`；
    3. 把这段文本拿去做 JSON 解析。
    """
    if not isinstance(model_text, str):
        return ""

    # 只保留第一个方括号到最后一个方括号之间的内容。
    start_index = model_text.find("[")
    end_index = model_text.rfind("]")
    if start_index < 0 or end_index < 0 or end_index < start_index:
        return ""
    return model_text[start_index:end_index + 1]


def normalize_single_item_text(item_text):
    """把单个物品名整理成稳定的短语格式。"""
    if not isinstance(item_text, str):
        return ""

    # 这里只做最基础的通用清洗：去首尾空格、转小写、合并多空格。
    clean_text = item_text.strip().lower()
    clean_text = clean_text.replace("-", " ")
    clean_text = re.sub(r"\s+", " ", clean_text)
    if not clean_text:
        return ""

    return clean_text


def normalize_model_output_items(item_list):
    """清洗模型返回的列表，只保留有效字符串。"""
    if not isinstance(item_list, list):
        return []

    normalized_items = []
    for item in item_list:
        # 每个元素都单独做最基础的文本清洗。
        normalized_item = normalize_single_item_text(item)
        if not normalized_item:
            continue
        normalized_items.append(normalized_item)
    return normalized_items


def parse_item_list_from_model_text(model_text):
    """把模型输出文本解析成最终的物品列表。"""
    # 先从整段文本里截出数组，再做 JSON 解析。
    json_array_text = extract_first_json_array_text(model_text)
    if not json_array_text:
        return []

    try:
        parsed_data = json.loads(json_array_text)
    except ValueError:
        return []

    # 最后再做一次 Python 端的轻量清洗。
    return normalize_model_output_items(parsed_data)


def parse_order_items_with_llm(order_text, api_core, model_name=GEMINI_MODEL, max_retry_times=MAX_RETRY_TIMES):
    """调用 LLM 解析订单，并返回统一结构。

    返回：
    1. success: 是否成功拿到有效物品列表；
    2. items: 解析得到的结果；
    3. error_msg: 失败原因，便于 action result 直接回传。
    """
    if not isinstance(order_text, str) or not order_text.strip():
        return False, [], "order text is empty"

    # 这里始终保留最后一次失败原因，方便 action result 直接回传给上游。
    last_error_msg = "llm did not return a valid item list"
    retry_hint_text = ""

    for _ in range(max_retry_times):
        # 每一轮都用当前 retry 提示重新拼完整 prompt。
        prompt_text = build_order_parse_prompt(order_text, retry_hint_text)
        api_result = api_core.generate_content(prompt_text, model_name)

        # 如果 HTTP 或接口层失败，直接记录错误并继续重试。
        if not api_result["ok"]:
            last_error_msg = api_result["error"] or "gemini request failed"
            continue

        # 如果模型明确返回了空数组，说明它判断句子里没有可提取信息。
        # 按当前任务要求，这种情况不继续重试，直接返回 no_info。
        json_array_text = extract_first_json_array_text(api_result["text"])
        if json_array_text:
            try:
                parsed_data = json.loads(json_array_text)
            except ValueError:
                parsed_data = None

            if parsed_data == []:
                return False, [], "no_info"

        # 如果接口成功，就尝试从模型文本里提取 JSON 数组。
        items = parse_item_list_from_model_text(api_result["text"])
        if items:
            return True, items, ""

        # 进入这里说明模型文字存在，但格式不对；下一轮追加更强的格式提醒。
        last_error_msg = "llm response format is invalid"
        retry_hint_text = RETRY_HINT_PROMPT

    return False, [], last_error_msg


class RestaurantOrderLlmActionServer:
    """餐厅题订单解析 action 服务端。"""

    def __init__(self):
        """初始化 action server 和 Gemini API core。"""
        from seu_restaurant_order_llm.msg import ParseRestaurantOrderAction

        # API core 只负责 HTTP 调用；业务判断都留在本文件里。
        self.api_core = GeminiApiCore()
        self.server = actionlib.SimpleActionServer(
            ACTION_NAME,
            ParseRestaurantOrderAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[restaurant_order_llm] action ready: %s", ACTION_NAME)

    def build_result_message(self, success, error_msg, items):
        """统一构造 action result，避免字段赋值散落在多个分支里。"""
        from seu_restaurant_order_llm.msg import ParseRestaurantOrderResult

        result = ParseRestaurantOrderResult()
        result.success = success
        result.error_msg = error_msg
        result.items = items
        return result

    def execute_cb(self, goal):
        """接收一句订单文本，调用 LLM 后返回物品列表。"""
        order_text = goal.order_text.strip()
        rospy.loginfo("[restaurant_order_llm] received order text: %s", order_text)

        # 如果上游在真正执行前就取消了目标，这里直接按 preempt 返回。
        if self.server.is_preempt_requested():
            result = self.build_result_message(False, "goal preempted before execution", [])
            self.server.set_preempted(result)
            return

        # 真正的订单解析逻辑集中在纯函数里，方便单独做单元测试。
        success, items, error_msg = parse_order_items_with_llm(order_text, self.api_core)
        result = self.build_result_message(success, error_msg, items)

        if success:
            rospy.loginfo("[restaurant_order_llm] parsed items: %s", items)
            self.server.set_succeeded(result)
            return

        rospy.logerr("[restaurant_order_llm] parse failed: %s", error_msg)
        self.server.set_aborted(result)


def main():
    """节点入口。"""
    # 这里只做 ROS 初始化和 spin，避免入口逻辑分散。
    rospy.init_node(NODE_NAME)
    RestaurantOrderLlmActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
