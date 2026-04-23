#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅题订单物品解析 action server。

职责：
1. 接收一整句英文订单文本；
2. 调用 LLM 做关键物品抽取；
3. 返回按说话顺序整理好的 `string[] items`；
4. 当前阶段只提供独立 action 节点，不改动状态机接线。
"""

import json
import os
import re
import sys

# 这里主动把当前脚本目录加入导入路径，保证直接 rosrun scripts 文件时导入稳定。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from gemini_api_core import GeminiApiCore
from qwen_api_core import QWEN_MODEL, QwenApiCore


# ROS 节点与 action 名称。
NODE_NAME = "restaurant_order_llm_action_server"
ACTION_NAME = "parse_restaurant_order"

# LLM provider、默认模型与请求重试次数。
LLM_PROVIDER_GEMINI = "gemini"
LLM_PROVIDER_QWEN = "qwen"
#选择llm常量
DEFAULT_LLM_PROVIDER = LLM_PROVIDER_QWEN

GEMINI_MODEL = "gemini-2.5-flash"
LLM_API_CORE_CLASS_BY_PROVIDER = {
    LLM_PROVIDER_GEMINI: GeminiApiCore,
    LLM_PROVIDER_QWEN: QwenApiCore,
}
LLM_MODEL_BY_PROVIDER = {
    LLM_PROVIDER_GEMINI: GEMINI_MODEL,
    LLM_PROVIDER_QWEN: QWEN_MODEL,
}
DEFAULT_LLM_MODEL = LLM_MODEL_BY_PROVIDER[DEFAULT_LLM_PROVIDER]
MAX_RETRY_TIMES = 3
DEFAULT_ITEM_COUNT = 1
ITEM_COUNT_PATTERN = re.compile(r"^(?P<name>.+?)_(?P<count>\d+)$")

# 规则 prompt：强约束模型只输出一个 JSON 数组。
# 当前餐厅题所有待抓取物品都来自上游闭集列表，不再允许开放集物品兜底。
ORDER_PARSE_RULE_PROMPT = """You are an item extraction module for a restaurant service robot.
Your job is to read one English customer order sentence and extract only the requested items that match the provided closed-set canonical item list.

Strict rules:
1. Output only one JSON array of strings.
2. Do not output explanations, markdown, code fences, or extra words.
3. Each string must use the format "canonical_item_name_count", for example "cola_2" means canonical item name "cola" with count 2.
4. Every item name in the output must be exactly one canonical item name from the closed-set list.
5. Never output an item name that is not in the closed-set list.
6. If a mentioned object cannot be confidently matched to one closed-set canonical item, ignore that object.
7. If no mentioned object can be confidently matched to the closed-set list, output [].
8. If the quantity is not clearly mentioned, use 1.
9. If the same canonical item appears multiple times, merge it into one entry and sum the quantity.
10. Keep item entries ordered by the first time the customer mentioned each item.
11. Remove quantity words and packaging helper words like bottle, bag, cup, can, pack, or piece when they are not part of the canonical item name.
12. The customer sentence comes from speech recognition and may contain small ASR errors, spelling errors, homophone errors, or strange words.
13. For simple English food or drink orders, infer the most likely intended closed-set item when the ASR text is close in sound, spelling, or meaning to a canonical item.
14. Examples of acceptable ASR correction: "ladies", "lace", or "lays chips" may refer to canonical "lays" if "lays" is in the closed-set list.
15. Do not invent unusual interpretations. If the intended item is still unclear after reasonable ASR correction, output [] or ignore only the unclear object.
16. The user may say simple sentences like "i want xxx and xxx", "please give me xxx", or "can i have two xxx and one yyy".

Output only one JSON array, for example: ["cola_2", "chips_1", "mineral water_1"]"""

# few-shot prompt：直接给模型示范输入和目标输出风格。
ORDER_PARSE_FEW_SHOT_PROMPT = """[Few-shot examples]
Customer: i want a bottle of cola and a bag of chips and a bottle of mineral water
Closed-set canonical item names: ["cola", "chips", "mineral water", "cookie"]
Output: ["cola_1", "chips_1", "mineral water_1"]

Customer: please give me two bottles of water and one cookie
Closed-set canonical item names: ["water", "cookie", "orange juice"]
Output: ["water_2", "cookie_1"]

Customer: can i have orange juice and a pack of crackers
Closed-set canonical item names: ["orange juice", "crackers", "bread"]
Output: ["orange juice_1", "crackers_1"]

Customer: i would like bread, chocolate milk and apple juice
Closed-set canonical item names: ["bread", "chocolate milk", "apple juice"]
Output: ["bread_1", "chocolate milk_1", "apple juice_1"]

Customer: i want cola and one more cola and two chips
Closed-set canonical item names: ["cola", "chips", "bread"]
Output: ["cola_2", "chips_2"]"""

# 当模型第一次格式不对时，第二次会补一段更硬的提醒。
RETRY_HINT_PROMPT = """[Retry instruction]
Your previous answer did not follow the required format.
Regenerate and output only one JSON array of strings in the format "canonical_item_name_count", for example "cola_2".
Every canonical_item_name must be copied exactly from the closed-set list.
If no closed-set item can be confidently extracted, output [].
"""


def build_llm_runtime(provider_name=DEFAULT_LLM_PROVIDER):
    """按 provider 构造当前 action server 使用的 API core 和模型名。"""
    normalized_provider_name = str(provider_name or "").strip().lower()
    api_core_class = LLM_API_CORE_CLASS_BY_PROVIDER.get(normalized_provider_name)
    model_name = LLM_MODEL_BY_PROVIDER.get(normalized_provider_name)
    if api_core_class is None or model_name is None:
        raise ValueError("unsupported llm provider: {}".format(provider_name))
    return api_core_class(), model_name


def normalize_closed_set_item_name(item_name):
    """把闭集物品名整理成稳定的标准格式。"""
    return normalize_single_item_text(item_name).rstrip(".")


def build_closed_set_item_name_list(closed_set_item_name_list):
    """把上游透传的闭集物品名列表标准化并去重。"""
    normalized_names = []
    for item_name in list(closed_set_item_name_list or []):
        normalized_name = normalize_closed_set_item_name(item_name)
        if normalized_name and normalized_name not in normalized_names:
            normalized_names.append(normalized_name)
    return normalized_names


def build_closed_set_prompt_text(closed_set_item_name_list):
    """把闭集约束拼成 prompt 段落。"""
    normalized_names = build_closed_set_item_name_list(closed_set_item_name_list)
    if not normalized_names:
        return ""

    return """[Closed-set canonical item names]
Canonical closed-set item names: {}
You must choose output item names only from the closed-set canonical item names above.
Use the exact spelling, spacing, and singular/plural form from the closed-set list.
If an ASR or homophone mistake most likely refers to a closed-set item, correct it to the most likely closed-set canonical name.
If the intended item is not in the closed set, ignore it. If no closed-set item remains, output [].""".format(
        json.dumps(normalized_names)
    )


def coerce_item_name_to_closed_set(item_name, closed_set_item_name_set=None):
    """只对闭集项做精确回落，不扩展开放集映射。"""
    if not item_name:
        return ""

    clean_item_name = normalize_single_item_text(item_name)
    if not clean_item_name or not closed_set_item_name_set:
        return clean_item_name

    if clean_item_name in closed_set_item_name_set:
        return clean_item_name

    singular_candidate = ""
    if len(clean_item_name) > 1 and clean_item_name.endswith("s"):
        singular_candidate = clean_item_name[:-1]
    if singular_candidate in closed_set_item_name_set:
        return singular_candidate

    return clean_item_name


def build_order_parse_prompt(order_text, retry_hint_text="", closed_set_item_name_list=None):
    """把规则、few-shot 和当前用户句子拼成一次完整 prompt。"""
    # 先放规则，保证模型一开始就知道只能输出 JSON 数组。
    prompt = ORDER_PARSE_RULE_PROMPT
    prompt += "\n\n"

    closed_set_prompt_text = build_closed_set_prompt_text(closed_set_item_name_list)
    if closed_set_prompt_text:
        prompt += closed_set_prompt_text
        prompt += "\n\n"

    # 再给 few-shot，让模型看到我们希望的输入输出样式。
    prompt += ORDER_PARSE_FEW_SHOT_PROMPT
    if closed_set_prompt_text:
        prompt += "\n\n"
        prompt += """Customer: please give me two drinks and one lays
Closed-set canonical item names: ["drink", "lays", "water"]
Output: ["drink_2", "lays_1"]

Customer: i want ladies and water
Closed-set canonical item names: ["lays", "water", "cola"]
Output: ["lays_1", "water_1"]

Customer: i want lace and a bottle of water
Closed-set canonical item names: ["lays", "water", "cola"]
Output: ["lays_1", "water_1"]

Customer: i want sandwich and dragon fruit
Closed-set canonical item names: ["cola", "lays", "water"]
Output: []

Customer: i want cola and something strange
Closed-set canonical item names: ["cola", "lays", "water"]
Output: ["cola_1"]"""
    prompt += "\n\n"

    if closed_set_prompt_text:
        prompt += "[Current closed-set reminder]\n"
        prompt += "For the current customer sentence, output item names only from this list: {}.\n\n".format(
            json.dumps(build_closed_set_item_name_list(closed_set_item_name_list))
        )

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
    clean_text = clean_text.replace("_", " ")
    clean_text = re.sub(r"\s+", " ", clean_text)
    if not clean_text:
        return ""

    return clean_text


def parse_item_count_entry(item_text, closed_set_item_name_set=None):
    """把模型输出的单个元素解析成 `(item_name, count)`。"""
    if not isinstance(item_text, str):
        return "", 0

    clean_text = item_text.strip().lower()
    clean_text = clean_text.replace("-", " ")
    clean_text = re.sub(r"\s*_\s*", "_", clean_text)
    clean_text = re.sub(r"\s+", " ", clean_text)
    if not clean_text:
        return "", 0

    item_count_match = ITEM_COUNT_PATTERN.match(clean_text)
    if item_count_match:
        item_name = normalize_single_item_text(item_count_match.group("name"))
        item_count = int(item_count_match.group("count"))
    else:
        item_name = normalize_single_item_text(clean_text)
        item_count = DEFAULT_ITEM_COUNT

    if not item_name or item_count <= 0:
        return "", 0
    item_name = coerce_item_name_to_closed_set(item_name, closed_set_item_name_set)
    return item_name, item_count


def normalize_model_output_items(item_list, closed_set_item_name_list=None):
    """清洗模型返回的列表，并合并成稳定的 `item_count` 结果。"""
    if not isinstance(item_list, list):
        return []

    closed_set_item_name_set = set(build_closed_set_item_name_list(closed_set_item_name_list))
    ordered_item_names = []
    item_count_map = {}

    for item in item_list:
        item_name, item_count = parse_item_count_entry(item, closed_set_item_name_set)
        if not item_name:
            continue
        if closed_set_item_name_set and item_name not in closed_set_item_name_set:
            continue
        if item_name not in item_count_map:
            ordered_item_names.append(item_name)
            item_count_map[item_name] = 0
        item_count_map[item_name] += item_count

    return ["{}_{}".format(item_name, item_count_map[item_name]) for item_name in ordered_item_names]


def parse_item_list_from_model_text(model_text, closed_set_item_name_list=None):
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
    return normalize_model_output_items(parsed_data, closed_set_item_name_list)


def parse_order_items_with_llm(
    order_text,
    api_core,
    model_name=DEFAULT_LLM_MODEL,
    max_retry_times=MAX_RETRY_TIMES,
    closed_set_item_name_list=None,
):
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
        prompt_text = build_order_parse_prompt(
            order_text,
            retry_hint_text=retry_hint_text,
            closed_set_item_name_list=closed_set_item_name_list,
        )
        api_result = api_core.generate_content(prompt_text, model_name)

        # 如果 HTTP 或接口层失败，直接记录错误并继续重试。
        if not api_result["ok"]:
            last_error_msg = api_result["error"] or "llm request failed"
            continue

        # 如果模型明确返回了空数组，说明它判断句子里没有可提取信息。
        # 按当前任务要求，这种情况不继续重试，直接返回 no_info。
        json_array_text = extract_first_json_array_text(api_result["text"])
        parsed_data = None
        if json_array_text:
            try:
                parsed_data = json.loads(json_array_text)
            except ValueError:
                parsed_data = None

            if parsed_data == []:
                return False, [], "no_info"

        # 如果接口成功，就尝试从模型文本里提取 JSON 数组。
        items = parse_item_list_from_model_text(
            api_result["text"],
            closed_set_item_name_list=closed_set_item_name_list,
        )
        if items:
            return True, items, ""

        # 进入这里说明模型文字存在但还没得到有效闭集物品；下一轮追加更强的格式提醒。
        if isinstance(parsed_data, list):
            last_error_msg = "no_info"
        else:
            last_error_msg = "llm response format is invalid"
        retry_hint_text = RETRY_HINT_PROMPT

    return False, [], last_error_msg


class RestaurantOrderLlmActionServer:
    """餐厅题订单解析 action 服务端。"""

    def __init__(self):
        """初始化 action server 和默认 LLM API core。"""
        import actionlib
        import rospy

        from seu_restaurant_msgs.msg import ParseRestaurantOrderAction

        # API core 只负责 HTTP 调用；业务判断都留在本文件里。
        self.api_core, self.model_name = build_llm_runtime(DEFAULT_LLM_PROVIDER)
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
        from seu_restaurant_msgs.msg import ParseRestaurantOrderResult

        result = ParseRestaurantOrderResult()
        result.success = success
        result.error_msg = error_msg
        result.items = items
        return result

    def execute_cb(self, goal):
        """接收一句订单文本，调用 LLM 后返回物品列表。"""
        import rospy

        order_text = goal.order_text.strip()
        closed_set_item_name_list = list(getattr(goal, "closed_set_item_name_list", []) or [])
        rospy.loginfo("[restaurant_order_llm] received order text: %s", order_text)

        # 如果上游在真正执行前就取消了目标，这里直接按 preempt 返回。
        if self.server.is_preempt_requested():
            result = self.build_result_message(False, "goal preempted before execution", [])
            self.server.set_preempted(result)
            return

        # 真正的订单解析逻辑集中在纯函数里，方便单独做单元测试。
        success, items, error_msg = parse_order_items_with_llm(
            order_text,
            self.api_core,
            model_name=self.model_name,
            closed_set_item_name_list=closed_set_item_name_list,
        )
        result = self.build_result_message(success, error_msg, items)

        if success:
            rospy.loginfo("[restaurant_order_llm] parsed items: %s", items)
            self.server.set_succeeded(result)
            return

        rospy.logerr("[restaurant_order_llm] parse failed: %s", error_msg)
        self.server.set_aborted(result)


def main():
    """节点入口。"""
    import rospy

    # 这里只做 ROS 初始化和 spin，避免入口逻辑分散。
    rospy.init_node(NODE_NAME)
    RestaurantOrderLlmActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
