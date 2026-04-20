#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订单解析核心逻辑测试。

说明：
1. 这里优先测试纯函数，避免依赖 ROS action 环境；
2. 这些测试先定义我们希望新节点具备的最小行为，再驱动后续实现。
"""

import os
import sys
import unittest


SCRIPT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "scripts",
)
SCRIPT_DIR = os.path.abspath(SCRIPT_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from gemini_api_core import GeminiApiCore  # pylint: disable=import-error
from qwen_api_core import QWEN_MODEL, QwenApiCore  # pylint: disable=import-error
from restaurant_order_llm_action_server import (  # pylint: disable=import-error
    DEFAULT_LLM_PROVIDER,
    GEMINI_MODEL,
    LLM_PROVIDER_GEMINI,
    LLM_PROVIDER_QWEN,
    build_order_parse_prompt,
    build_llm_runtime,
    normalize_model_output_items,
    parse_item_count_entry,
    parse_item_list_from_model_text,
    parse_order_items_with_llm,
)


class FakeApiCore:
    """用固定返回值替代真实 LLM 请求。"""

    def __init__(self, response_list):
        self.response_list = response_list
        self.prompt_list = []

    def generate_content(self, prompt_text, model_name):
        del model_name
        self.prompt_list.append(prompt_text)
        response_index = len(self.prompt_list) - 1
        return self.response_list[response_index]


class OrderParserCoreTest(unittest.TestCase):
    """验证订单解析节点中最关键的纯逻辑。"""

    def test_build_llm_runtime_should_default_to_qwen(self):
        """默认 provider 应该是 qwen，并返回对应 API core 与模型名。"""
        api_core, model_name = build_llm_runtime()

        self.assertEqual(DEFAULT_LLM_PROVIDER, LLM_PROVIDER_QWEN)
        self.assertIsInstance(api_core, QwenApiCore)
        self.assertEqual(model_name, QWEN_MODEL)

    def test_build_llm_runtime_should_support_gemini(self):
        """显式切到 gemini 时，应返回 Gemini API core 与模型名。"""
        api_core, model_name = build_llm_runtime(LLM_PROVIDER_GEMINI)

        self.assertIsInstance(api_core, GeminiApiCore)
        self.assertEqual(model_name, GEMINI_MODEL)

    def test_build_order_parse_prompt_should_contain_closed_set_rules_and_examples(self):
        """prompt 里必须包含当前句子、few-shot，以及闭集约束。"""
        sentence = "i want a bottle of cola and a bag of chips"
        prompt = build_order_parse_prompt(
            sentence,
            closed_set_item_name_list=[" Cola ", "lays", "water", "drink", "milk", "lays"],
        )
        self.assertIn(sentence, prompt)
        self.assertIn("Few-shot examples", prompt)
        self.assertIn("Output only one JSON array", prompt)
        self.assertIn("item_name_count", prompt)
        self.assertIn("Closed-set canonical item names", prompt)
        self.assertIn('["cola", "lays", "water", "drink", "milk"]', prompt)
        self.assertIn('["drink_2", "lays_1"]', prompt)
        self.assertIn('["lays_1", "water_1"]', prompt)

    def test_parse_item_list_from_model_text_should_extract_json_array(self):
        """模型返回标准 JSON 数组时，应解析出规范化后的 item_count 列表。"""
        text = '["cola_2", " chips_1 ", "mineral water_1"]'
        self.assertEqual(
            parse_item_list_from_model_text(text),
            ["cola_2", "chips_1", "mineral water_1"],
        )

    def test_parse_item_list_from_model_text_should_ignore_wrapping_text(self):
        """即使模型前后夹带少量废话，也应尽量提取第一个数组。"""
        text = 'Result: ["cola_2", "chips_1", "mineral water_1"] thanks'
        self.assertEqual(
            parse_item_list_from_model_text(text),
            ["cola_2", "chips_1", "mineral water_1"],
        )

    def test_parse_item_count_entry_should_default_count_to_one_when_missing(self):
        """模型漏掉数量后，Python 端默认补成 1。"""
        self.assertEqual(
            parse_item_count_entry("Cola"),
            ("cola", 1),
        )

    def test_normalize_model_output_items_should_drop_invalid_values_and_merge_duplicates(self):
        """规范化阶段应过滤无效值，并按首次出现顺序合并同类物品数量。"""
        self.assertEqual(
            normalize_model_output_items([" Cola_2 ", "", None, "cola", "mineral water_1"]),
            ["cola_3", "mineral water_1"],
        )

    def test_normalize_model_output_items_should_coerce_simple_plural_only_for_closed_set(self):
        """闭集项简单复数应回落为标准名，开放集保持原样。"""
        self.assertEqual(
            normalize_model_output_items(
                [" drinks_2 ", "orange juice_1", "drink_1", "lays_1"],
                ["cola", "lays", "water", "drink", "milk"],
            ),
            ["drink_3", "orange juice_1", "lays_1"],
        )

    def test_parse_order_items_with_llm_should_return_items_when_response_is_valid(self):
        """一次有效返回时，应直接给出最终 item_count 列表。"""
        fake_api_core = FakeApiCore([
            {
                "ok": True,
                "text": '["drinks_2", "lays_1", "orange juice_1"]',
                "error": "",
            }
        ])
        success, items, error_msg = parse_order_items_with_llm(
            "i want cola chips and mineral water",
            fake_api_core,
            closed_set_item_name_list=["cola", "lays", "water", "drink", "milk"],
        )
        self.assertTrue(success)
        self.assertEqual(items, ["drink_2", "lays_1", "orange juice_1"])
        self.assertEqual(error_msg, "")
        self.assertIn('["cola", "lays", "water", "drink", "milk"]', fake_api_core.prompt_list[0])

    def test_parse_order_items_with_llm_should_retry_after_invalid_format(self):
        """第一次格式不合法时，第二次应带 retry 提示继续请求。"""
        fake_api_core = FakeApiCore([
            {
                "ok": True,
                "text": 'cola and chips',
                "error": "",
            },
            {
                "ok": True,
                "text": '["cola_1", "chips_1"]',
                "error": "",
            },
        ])
        success, items, error_msg = parse_order_items_with_llm(
            "i want cola and chips",
            fake_api_core,
            closed_set_item_name_list=["cola", "lays", "water", "drink", "milk"],
        )
        self.assertTrue(success)
        self.assertEqual(items, ["cola_1", "chips_1"])
        self.assertEqual(error_msg, "")
        self.assertEqual(len(fake_api_core.prompt_list), 2)
        self.assertIn("Retry instruction", fake_api_core.prompt_list[1])
        self.assertIn("Closed-set canonical item names", fake_api_core.prompt_list[1])

    def test_parse_order_items_with_llm_should_return_no_info_when_model_returns_empty_list(self):
        """模型明确返回空数组时，应直接失败并返回 no_info。"""
        fake_api_core = FakeApiCore([
            {
                "ok": True,
                "text": "[]",
                "error": "",
            }
        ])
        success, items, error_msg = parse_order_items_with_llm(
            "i want nothing useful",
            fake_api_core,
        )
        self.assertFalse(success)
        self.assertEqual(items, [])
        self.assertEqual(error_msg, "no_info")
        self.assertEqual(len(fake_api_core.prompt_list), 1)


if __name__ == "__main__":
    unittest.main()
