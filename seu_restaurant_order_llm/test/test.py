#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直连 Qwen 的纯 Python 测试脚本。"""

import os
import sys
import time


TEST_ORDER_TEXT = "i want a bottle of cola, a bag of chip and a bottle of mineral water"

SCRIPT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "scripts",
)
SCRIPT_DIR = os.path.abspath(SCRIPT_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from restaurant_order_llm_action_server import (  # pylint: disable=import-error
    LLM_PROVIDER_QWEN,
    build_llm_runtime,
    build_order_parse_prompt,
    parse_item_list_from_model_text,
)


def main():
    """固定一句订单，直接调用 Qwen 并打印返回与耗时。"""
    api_core, model_name = build_llm_runtime(LLM_PROVIDER_QWEN)
    prompt_text = build_order_parse_prompt(TEST_ORDER_TEXT)

    start_time = time.perf_counter()
    api_result = api_core.generate_content(prompt_text, model_name)
    elapsed_sec = time.perf_counter() - start_time

    model_text = api_result.get("text", "")
    parsed_items = parse_item_list_from_model_text(model_text)

    print("provider:", LLM_PROVIDER_QWEN)
    print("model:", model_name)
    print("order:", TEST_ORDER_TEXT)
    print("elapsed_sec: {:.3f}".format(elapsed_sec))
    print("http_status:", api_result.get("status_code"))
    print("ok:", api_result.get("ok"))
    print("error_code:", api_result.get("error_code"))
    print("error:", api_result.get("error"))
    if api_result.get("reasoning_text"):
        print("reasoning_text:", api_result.get("reasoning_text"))
    print("model_text:", model_text)
    print("parsed_items:", parsed_items)


if __name__ == "__main__":
    main()
