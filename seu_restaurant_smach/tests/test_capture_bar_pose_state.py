#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试启动时记录回吧台位姿的状态。"""

import importlib.util
import sys
import types
import unittest
from unittest.mock import patch
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "restaurant_smach.py"


class DummyPoseStamped:
    pass


class DummyOdometry:
    pass


class DummyNavGoal:
    TARGET_MODE_POSE = 0
    TARGET_MODE_NAME = 1

    def __init__(self):
        self.target_mode = None
        self.nav_pose = None
        self.target_name = ""
        self.timeout_sec = 0.0


class FakeState:
    def __init__(self, outcomes=None, input_keys=None, output_keys=None):
        self.outcomes = outcomes or []
        self.input_keys = input_keys or []
        self.output_keys = output_keys or []


class FakeStateMachine:
    def __init__(self, outcomes=None):
        self.outcomes = outcomes or []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    @staticmethod
    def add(*args, **kwargs):
        return None


class FakeActionClient:
    def __init__(self, *args, **kwargs):
        pass


def _make_module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def load_module(current_pose, reversed_pose, calls):
    fake_odom = DummyOdometry()

    def wait_for_message(topic, msg_type, timeout=None):
        calls['wait_for_message'] = (topic, msg_type, timeout)
        return fake_odom

    def odometry_to_pose_stamped(odom):
        calls['odometry_to_pose_stamped'] = odom
        return current_pose

    def reverse_pose_orientation(pose):
        calls['reverse_pose_orientation'] = pose
        return reversed_pose

    fake_rospy = _make_module(
        'rospy',
        wait_for_message=wait_for_message,
        wait_for_service=lambda *args, **kwargs: None,
        ServiceProxy=lambda *args, **kwargs: types.SimpleNamespace(call=lambda request: types.SimpleNamespace(state=True, errormsg='')),
        Duration=lambda secs: secs,
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
        init_node=lambda *args, **kwargs: None,
    )
    fake_smach = _make_module('smach', State=FakeState, StateMachine=FakeStateMachine)
    fake_smach_ros = _make_module('smach_ros', IntrospectionServer=lambda *args, **kwargs: types.SimpleNamespace(start=lambda: None, stop=lambda: None))
    fake_actionlib = _make_module('actionlib', SimpleActionClient=FakeActionClient)
    fake_geometry_msgs_msg = _make_module('geometry_msgs.msg', PoseStamped=DummyPoseStamped)
    fake_geometry_msgs = _make_module('geometry_msgs', msg=fake_geometry_msgs_msg)
    fake_nav_msgs_msg = _make_module('nav_msgs.msg', Odometry=DummyOdometry)
    fake_nav_msgs = _make_module('nav_msgs', msg=fake_nav_msgs_msg)
    fake_restaurant_msgs_msg = _make_module(
        'seu_restaurant_msgs.msg',
        AlignAndTakeOrderAction=object,
        AlignAndTakeOrderGoal=lambda **kwargs: kwargs,
        FindWavingCustomerAction=object,
        FindWavingCustomerGoal=lambda **kwargs: kwargs,
        RestaurantNavAction=object,
        RestaurantNavGoal=DummyNavGoal,
    )
    fake_restaurant_msgs = _make_module('seu_restaurant_msgs', msg=fake_restaurant_msgs_msg)
    fake_voice_srv = _make_module(
        'seu_voice.srv',
        seu_voice=object,
        seu_voiceRequest=lambda text='': types.SimpleNamespace(text=text),
    )
    fake_voice = _make_module('seu_voice', srv=fake_voice_srv)
    fake_restaurant_states = _make_module(
        'restaurant_states',
        build_customer_nav_pose=lambda *args, **kwargs: None,
        odometry_to_pose_stamped=odometry_to_pose_stamped,
        reverse_pose_orientation=reverse_pose_orientation,
    )

    spec = importlib.util.spec_from_file_location('restaurant_smach_under_test', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    with patch.dict(
        sys.modules,
        {
            'rospy': fake_rospy,
            'smach': fake_smach,
            'smach_ros': fake_smach_ros,
            'actionlib': fake_actionlib,
            'geometry_msgs': fake_geometry_msgs,
            'geometry_msgs.msg': fake_geometry_msgs_msg,
            'nav_msgs': fake_nav_msgs,
            'nav_msgs.msg': fake_nav_msgs_msg,
            'seu_restaurant_msgs': fake_restaurant_msgs,
            'seu_restaurant_msgs.msg': fake_restaurant_msgs_msg,
            'seu_voice': fake_voice,
            'seu_voice.srv': fake_voice_srv,
            'restaurant_states': fake_restaurant_states,
        },
    ):
        spec.loader.exec_module(module)
    return module, fake_odom


class CaptureBarPoseStateTest(unittest.TestCase):
    """启动位姿捕获状态测试。"""

    def test_capture_bar_pose_state_reads_pose_and_saves_reversed_pose(self):
        """启动状态应读取当前位姿，并把反向后的位姿写入 bar_pose_map。"""
        current_pose = types.SimpleNamespace(name='current_pose')
        reversed_pose = types.SimpleNamespace(name='reversed_pose')
        calls = {}

        module, fake_odom = load_module(current_pose, reversed_pose, calls)
        state = module.CaptureBarPoseState()
        userdata = types.SimpleNamespace()

        outcome = state.execute(userdata)

        self.assertEqual(outcome, 'success')
        self.assertIs(userdata.bar_pose_map, reversed_pose)
        self.assertEqual(calls['wait_for_message'][0], '/map_to_baselink')
        self.assertIs(calls['odometry_to_pose_stamped'], fake_odom)
        self.assertIs(calls['reverse_pose_orientation'], current_pose)


if __name__ == '__main__':
    unittest.main()
