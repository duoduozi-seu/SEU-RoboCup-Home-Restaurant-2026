#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试挥手候选目标筛选逻辑。"""

import os
import sys
import unittest


WORKSPACE_SRC = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src"
SCRIPT_DIR = os.path.join(WORKSPACE_SRC, "seu_restaurant_wave", "scripts")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from wave_detector_core import select_nearest_waving_candidate


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


if __name__ == "__main__":
    unittest.main()
