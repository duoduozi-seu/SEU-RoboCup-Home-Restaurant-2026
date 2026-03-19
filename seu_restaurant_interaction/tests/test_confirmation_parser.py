#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试 yes/no 确认语句解析逻辑。"""

import os
import sys
import unittest


WORKSPACE_SRC = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src"
SCRIPT_DIR = os.path.join(WORKSPACE_SRC, "seu_restaurant_interaction", "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from person_align_core import classify_confirmation_text, select_nearest_person_within_range


class ConfirmationParserTest(unittest.TestCase):
    """确认词解析测试。"""

    def test_classify_confirmation_text_accepts_yes_variants(self):
        """常见肯定短语应被判为确认成功。"""
        self.assertEqual(classify_confirmation_text("yes that's right"), "yes")
        self.assertEqual(classify_confirmation_text("okay correct"), "yes")
        self.assertEqual(classify_confirmation_text("yeah"), "yes")

    def test_classify_confirmation_text_accepts_no_variants(self):
        """常见否定短语应被判为需要重问。"""
        self.assertEqual(classify_confirmation_text("no"), "no")
        self.assertEqual(classify_confirmation_text("nope that's wrong"), "no")
        self.assertEqual(classify_confirmation_text("incorrect order"), "no")

    def test_classify_confirmation_text_returns_unknown_for_irrelevant_sentence(self):
        """与确认无关的句子不应被误判成 yes/no。"""
        self.assertEqual(classify_confirmation_text("two colas and one burger"), "unknown")

    def test_select_nearest_person_within_range_ignores_far_people(self):
        """微调阶段应只返回阈值内最近的人。"""
        candidates = [
            {"depth_m": 2.6, "center_x": 210},
            {"depth_m": 1.4, "center_x": 320},
            {"depth_m": 1.8, "center_x": 280},
        ]

        selected = select_nearest_person_within_range(candidates, 2.0)

        self.assertEqual(selected["depth_m"], 1.4)


if __name__ == "__main__":
    unittest.main()
