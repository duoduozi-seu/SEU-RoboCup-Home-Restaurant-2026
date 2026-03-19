#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试挥手候选目标筛选与调试图像逻辑。"""

import os
import sys
import unittest

import numpy as np


WORKSPACE_SRC = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src"
SCRIPT_DIR = os.path.join(WORKSPACE_SRC, "seu_restaurant_wave", "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from wave_detector_core import render_debug_image, select_nearest_waving_candidate


class WaveGeometryTest(unittest.TestCase):
    """挥手目标选择测试。"""

    def test_select_nearest_waving_candidate_prefers_smallest_positive_depth(self):
        """多人同时挥手时应选择最近且深度有效的目标。"""
        candidates = [
            {"is_waving": True, "depth_m": 2.4, "person_point_map": (2.0, 0.0, 0.0)},
            {"is_waving": True, "depth_m": 1.2, "person_point_map": (1.0, 0.0, 0.0)},
            {"is_waving": False, "depth_m": 0.8, "person_point_map": (0.5, 0.0, 0.0)},
        ]

        selected = select_nearest_waving_candidate(candidates)

        self.assertEqual(selected["depth_m"], 1.2)

    def test_select_nearest_waving_candidate_ignores_invalid_depth(self):
        """深度无效的挥手目标不能被选中。"""
        candidates = [
            {"is_waving": True, "depth_m": 0.0, "person_point_map": (2.0, 0.0, 0.0)},
            {"is_waving": True, "depth_m": -1.0, "person_point_map": (1.0, 0.0, 0.0)},
        ]

        self.assertIsNone(select_nearest_waving_candidate(candidates))

    def test_render_debug_image_keeps_image_shape(self):
        """调试图像渲染后应保持原始尺寸不变。"""
        image = np.zeros((64, 96, 3), dtype=np.uint8)
        candidate = {"box": (10, 8, 40, 50), "depth_m": 1.25, "is_waving": True}

        rendered = render_debug_image(image, [candidate], candidate)

        self.assertEqual(rendered.shape, image.shape)

    def test_render_debug_image_marks_selected_candidate(self):
        """存在目标时，调试图像应被画上可见标记。"""
        image = np.zeros((64, 96, 3), dtype=np.uint8)
        candidate = {"box": (10, 8, 40, 50), "depth_m": 1.25, "is_waving": True}

        rendered = render_debug_image(image, [candidate], candidate)

        self.assertGreater(int(rendered.sum()), 0)


if __name__ == "__main__":
    unittest.main()
