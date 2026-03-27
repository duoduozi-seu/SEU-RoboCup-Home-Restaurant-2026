#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''测试餐厅题导航 wrapper 的轻量逻辑。'''

import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from nav_action_core import resolve_waypoint_by_name


class NavActionCoreTest(unittest.TestCase):
    '''导航包装层轻量逻辑测试。'''

    def test_resolve_waypoint_by_name_supports_case_and_underscore_fallback(self):
        '''宽松匹配应兼容大小写和下划线/空格差异。'''
        waypoints = {
            'waypoints': {
                'StartPoint': {'nav_pose': {'position': {}, 'orientation': {}}, 'pid_pose': {'position': {}, 'orientation': {}}},
                'Kitchen_0': {'nav_pose': {'position': {}, 'orientation': {}}, 'pid_pose': {'position': {}, 'orientation': {}}},
            }
        }

        matched_name, waypoint = resolve_waypoint_by_name(waypoints, 'kitchen 0')

        self.assertEqual(matched_name, 'Kitchen_0')
        self.assertIn('nav_pose', waypoint)

    def test_resolve_waypoint_by_name_raises_for_unknown_name(self):
        '''未知航点名应明确抛出 KeyError。'''
        with self.assertRaises(KeyError):
            resolve_waypoint_by_name({'waypoints': {'Bar': {}}}, 'table 9')


if __name__ == '__main__':
    unittest.main()
