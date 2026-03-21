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

from restaurant_order_llm_action_server import (  # pylint: disable=import-error
    build_order_parse_prompt,
    normalize_model_output_items,
    parse_item_list_from_model_text,
    parse_order_items_with_llm,
)


class FakeApiCore:
    """用固定返回值替代真实 Gemini 请求。"""

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

    def test_build_order_parse_prompt_should_contain_user_text_and_few_shot(self):
        """prompt 里必须同时包含当前句子和 few-shot 示例。"""
        sentence = "i want a bottle of cola and a bag of chips"
        prompt = build_order_parse_prompt(sentence)
        self.assertIn(sentence, prompt)
        self.assertIn("Few-shot examples", prompt)
        self.assertIn("Output only one JSON array", prompt)

    def test_parse_item_list_from_model_text_should_extract_json_array(self):
        """模型返回标准 JSON 数组时，应解析出规范化列表。"""
        text = '["cola", " chips ", "mineral water"]'
        self.assertEqual(
            parse_item_list_from_model_text(text),
            ["cola", "chips", "mineral water"],
        )

    def test_parse_item_list_from_model_text_should_ignore_wrapping_text(self):
        """即使模型前后夹带少量废话，也应尽量提取第一个数组。"""
        text = 'Result: ["cola", "chips", "mineral water"] thanks'
        self.assertEqual(
            parse_item_list_from_model_text(text),
            ["cola", "chips", "mineral water"],
        )

    def test_normalize_model_output_items_should_drop_invalid_values(self):
        """规范化阶段应过滤空串和非字符串元素。"""
        self.assertEqual(
            normalize_model_output_items([" Cola ", "", None, "mineral water"]),
            ["cola", "mineral water"],
        )

    def test_parse_order_items_with_llm_should_return_items_when_response_is_valid(self):
        """一次有效返回时，应直接给出最终物品列表。"""
        fake_api_core = FakeApiCore([
            {
                "ok": True,
                "text": '["cola", "chips", "mineral water"]',
                "error": "",
            }
        ])
        success, items, error_msg = parse_order_items_with_llm(
            "i want cola chips and mineral water",
            fake_api_core,
        )
        self.assertTrue(success)
        self.assertEqual(items, ["cola", "chips", "mineral water"])
        self.assertEqual(error_msg, "")

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
                "text": '["cola", "chips"]',
                "error": "",
            },
        ])
        success, items, error_msg = parse_order_items_with_llm(
            "i want cola and chips",
            fake_api_core,
        )
        self.assertTrue(success)
        self.assertEqual(items, ["cola", "chips"])
        self.assertEqual(error_msg, "")
        self.assertEqual(len(fake_api_core.prompt_list), 2)
        self.assertIn("Retry instruction", fake_api_core.prompt_list[1])

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
