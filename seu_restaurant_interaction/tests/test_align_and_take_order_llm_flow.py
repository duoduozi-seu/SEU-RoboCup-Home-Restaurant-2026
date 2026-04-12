#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试 interaction 节点中的 LLM 订单解析分支。"""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "align_and_take_order_action_server.py"


class DummyPoseStamped:
    pass


class DummyOdometry:
    pass


class DummyTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class DummyAlignAndTakeOrderResult:
    def __init__(self, success=False, error_msg="", order_text="", rounds_used=0, orderlist=None, orderlist_num=None):
        self.success = success
        self.error_msg = error_msg
        self.order_text = order_text
        self.rounds_used = rounds_used
        self.orderlist = list(orderlist or [])
        self.orderlist_num = list(orderlist_num or [])
        self.service_pose_map = None


class DummyParseRestaurantOrderGoal:
    def __init__(self):
        self.order_text = ""


class FakeLlmActionClient:
    planned_results = []
    created_clients = []

    def __init__(self, action_name, action_type):
        del action_type
        self.action_name = action_name
        self.goal_list = []
        self.result_index = 0
        FakeLlmActionClient.created_clients.append(self)

    def wait_for_server(self, timeout=None):
        del timeout
        return True

    def send_goal(self, goal):
        self.goal_list.append(goal)

    def wait_for_result(self):
        return True

    def get_result(self):
        result = FakeLlmActionClient.planned_results[self.result_index]
        self.result_index += 1
        return result


class FakeActionServer:
    def __init__(self):
        self.succeeded_result = None
        self.aborted_result = None
        self.preempted_result = None
        self.preempt_requested = False

    def is_preempt_requested(self):
        return self.preempt_requested

    def set_succeeded(self, result):
        self.succeeded_result = result

    def set_aborted(self, result, text=None):
        del text
        self.aborted_result = result

    def set_preempted(self, result=None):
        self.preempted_result = result


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
        Subscriber=lambda *args, **kwargs: None,
        Publisher=lambda *args, **kwargs: types.SimpleNamespace(publish=lambda msg: None),
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
        init_node=lambda *args, **kwargs: None,
        spin=lambda: None,
        is_shutdown=lambda: False,
        Duration=lambda secs: secs,
        Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace()),
        Rate=lambda hz: types.SimpleNamespace(sleep=lambda: None),
    )
    fake_actionlib = _make_module(
        'actionlib',
        SimpleActionServer=object,
        SimpleActionClient=FakeLlmActionClient,
    )
    fake_tf2_ros = _make_module(
        'tf2_ros',
        Buffer=lambda: types.SimpleNamespace(),
        TransformListener=lambda buffer: types.SimpleNamespace(buffer=buffer),
    )
    fake_geometry_msgs_msg = _make_module('geometry_msgs.msg', PoseStamped=DummyPoseStamped, Twist=DummyTwist)
    fake_geometry_msgs = _make_module('geometry_msgs', msg=fake_geometry_msgs_msg)
    fake_nav_msgs_msg = _make_module('nav_msgs.msg', Odometry=DummyOdometry)
    fake_nav_msgs = _make_module('nav_msgs', msg=fake_nav_msgs_msg)
    fake_restaurant_msgs_msg = _make_module(
        'seu_restaurant_msgs.msg',
        AlignAndTakeOrderAction=object,
        AlignAndTakeOrderResult=DummyAlignAndTakeOrderResult,
        ParseRestaurantOrderAction=object,
        ParseRestaurantOrderGoal=DummyParseRestaurantOrderGoal,
    )
    fake_restaurant_msgs = _make_module('seu_restaurant_msgs', msg=fake_restaurant_msgs_msg)
    fake_speech_srv = _make_module(
        'seu_speech_recognition.srv',
        seu_speech_recognition=object,
        seu_speech_recognitionRequest=lambda enable=False, task='': types.SimpleNamespace(enable=enable, task=task),
    )
    fake_speech = _make_module('seu_speech_recognition', srv=fake_speech_srv)
    fake_voice_srv = _make_module(
        'seu_voice.srv',
        seu_voice=object,
        seu_voiceRequest=lambda text='': types.SimpleNamespace(text=text),
    )
    fake_voice = _make_module('seu_voice', srv=fake_voice_srv)
    fake_person_align_core = _make_module(
        'person_align_core',
        classify_confirmation_text=lambda text: 'yes' if text == 'yes' else 'no' if text == 'no' else 'unknown',
        select_nearest_person_within_range=lambda candidates, max_distance: None,
    )

    spec = importlib.util.spec_from_file_location('align_and_take_order_action_server_under_test', MODULE_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    with patch.dict(
        sys.modules,
        {
            'rospy': fake_rospy,
            'actionlib': fake_actionlib,
            'tf2_ros': fake_tf2_ros,
            'geometry_msgs': fake_geometry_msgs,
            'geometry_msgs.msg': fake_geometry_msgs_msg,
            'nav_msgs': fake_nav_msgs,
            'nav_msgs.msg': fake_nav_msgs_msg,
            'seu_restaurant_msgs': fake_restaurant_msgs,
            'seu_restaurant_msgs.msg': fake_restaurant_msgs_msg,
            'seu_speech_recognition': fake_speech,
            'seu_speech_recognition.srv': fake_speech_srv,
            'seu_voice': fake_voice,
            'seu_voice.srv': fake_voice_srv,
            'person_align_core': fake_person_align_core,
        },
    ):
        spec.loader.exec_module(module)
    return module


class AlignAndTakeOrderLlmFlowTest(unittest.TestCase):
    """验证 interaction 节点中新增的 LLM 订单解析流程。"""

    def setUp(self):
        FakeLlmActionClient.planned_results = []
        FakeLlmActionClient.created_clients = []
        self.module = load_module()

    def build_server(self, listen_outputs):
        server = object.__new__(self.module.AlignAndTakeOrderActionServer)
        server.current_pose = types.SimpleNamespace(name='pose')
        server.server = FakeActionServer()
        server.cmd_vel_pub = types.SimpleNamespace(publish=lambda msg: None)
        server.voice_client = None
        server.speech_client = None
        server.tf_buffer = types.SimpleNamespace()
        server.tf_listener = types.SimpleNamespace()
        server._align_to_customer = lambda timeout_sec: True
        server._get_current_pose = lambda: types.SimpleNamespace(name='pose')
        server._stop_robot = lambda: None
        server._say = lambda text: None
        output_iter = iter(listen_outputs)
        server._listen_once = lambda: next(output_iter)
        return server

    def test_execute_cb_should_retry_whole_order_when_llm_returns_no_info(self):
        """LLM 返回 no_info 时，应重新回到让主人说指令的流程。"""
        FakeLlmActionClient.planned_results = [
            types.SimpleNamespace(success=False, error_msg='no_info', items=[]),
            types.SimpleNamespace(success=True, error_msg='', items=['cola_2', 'chips_1']),
        ]
        server = self.build_server([
            'first unclear order', 'yes',
            'second clear order', 'yes',
        ])
        goal = types.SimpleNamespace(ask_text='ask', max_rounds=2, align_timeout_sec=1.0, dialog_timeout_sec=1.0)

        server.execute_cb(goal)

        self.assertIsNotNone(server.server.succeeded_result)
        self.assertIsNone(server.server.aborted_result)
        self.assertEqual(server.server.succeeded_result.order_text, 'second clear order')
        self.assertEqual(server.server.succeeded_result.orderlist, ['cola', 'chips'])
        self.assertEqual(server.server.succeeded_result.orderlist_num, [2, 1])
        self.assertEqual(server.server.succeeded_result.rounds_used, 2)

    def test_execute_cb_should_abort_when_llm_returns_invalid_item_count_list(self):
        """LLM 成功但返回非法 item_count 列表时，应直接失败退出。"""
        FakeLlmActionClient.planned_results = [
            types.SimpleNamespace(success=True, error_msg='', items=['cola_0']),
        ]
        server = self.build_server(['order text', 'yes'])
        goal = types.SimpleNamespace(ask_text='ask', max_rounds=1, align_timeout_sec=1.0, dialog_timeout_sec=1.0)

        server.execute_cb(goal)

        self.assertIsNone(server.server.succeeded_result)
        self.assertIsNotNone(server.server.aborted_result)
        self.assertEqual(
            server.server.aborted_result.error_msg,
            'llm parse failed or network error: invalid item_count list',
        )

    def test_execute_cb_should_abort_when_llm_returns_non_no_info_error(self):
        """LLM 返回非 no_info 错误时，应直接失败退出。"""
        FakeLlmActionClient.planned_results = [
            types.SimpleNamespace(success=False, error_msg='HTTP 503: upstream error', items=[]),
        ]
        server = self.build_server(['order text', 'yes'])
        goal = types.SimpleNamespace(ask_text='ask', max_rounds=1, align_timeout_sec=1.0, dialog_timeout_sec=1.0)

        server.execute_cb(goal)

        self.assertIsNone(server.server.succeeded_result)
        self.assertIsNotNone(server.server.aborted_result)
        self.assertEqual(
            server.server.aborted_result.error_msg,
            'llm parse failed or network error: HTTP 503: upstream error',
        )


if __name__ == '__main__':
    unittest.main()
