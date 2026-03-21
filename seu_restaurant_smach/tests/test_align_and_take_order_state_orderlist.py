#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试 SMACH 中接单状态对 orderlist 的透传。"""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


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
        self.target_name = ''
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
    next_result = None

    def __init__(self, *args, **kwargs):
        self.goal = None

    def wait_for_server(self, duration):
        del duration
        return True

    def send_goal(self, goal):
        self.goal = goal

    def wait_for_result(self):
        return True

    def get_result(self):
        return FakeActionClient.next_result


def _make_module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def load_module():
    fake_rospy = _make_module(
        'rospy',
        wait_for_message=lambda *args, **kwargs: DummyOdometry(),
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
        AlignAndTakeOrderGoal=lambda **kwargs: types.SimpleNamespace(**kwargs),
        FindWavingCustomerAction=object,
        FindWavingCustomerGoal=lambda **kwargs: types.SimpleNamespace(**kwargs),
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
        odometry_to_pose_stamped=lambda odom: DummyPoseStamped(),
        reverse_pose_orientation=lambda pose: pose,
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
    return module


class AlignAndTakeOrderStateOrderListTest(unittest.TestCase):
    """验证接单状态会把 orderlist 写入 userdata。"""

    def test_execute_should_copy_orderlist_to_userdata(self):
        module = load_module()
        FakeActionClient.next_result = types.SimpleNamespace(
            success=True,
            service_pose_map=types.SimpleNamespace(name='service_pose'),
            order_text='cola and chips',
            orderlist=['cola', 'chips'],
        )

        state = module.AlignAndTakeOrderState()
        userdata = types.SimpleNamespace()
        outcome = state.execute(userdata)

        self.assertEqual(outcome, 'success')
        self.assertEqual(state.output_keys, ['service_pose_map', 'order_text', 'orderlist'])
        self.assertEqual(userdata.order_text, 'cola and chips')
        self.assertEqual(userdata.orderlist, ['cola', 'chips'])


if __name__ == '__main__':
    unittest.main()
