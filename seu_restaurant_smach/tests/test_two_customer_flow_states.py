#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试双人服务调度相关的状态逻辑。"""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "restaurant_smach.py"


class DummyPoseStamped:
    pass


class DummyNavGoal:
    TARGET_MODE_POSE = 2
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
    def __init__(self, outcomes=None, input_keys=None, output_keys=None):
        self.outcomes = outcomes or []
        self.input_keys = input_keys or []
        self.output_keys = output_keys or []
        self.userdata = types.SimpleNamespace()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    @staticmethod
    def add(*args, **kwargs):
        return None


class FakeActionClient:
    next_result = None
    instances = []

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.goals = []
        FakeActionClient.instances.append(self)

    def wait_for_server(self, duration):
        del duration
        return True

    def send_goal(self, goal):
        self.goals.append(goal)

    def wait_for_result(self):
        return True

    def get_result(self):
        return FakeActionClient.next_result

    def get_goal_status_text(self):
        return ""


def _make_module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def load_module():
    FakeActionClient.instances = []
    FakeActionClient.next_result = None

    fake_rospy = _make_module(
        "rospy",
        wait_for_service=lambda *args, **kwargs: None,
        ServiceProxy=lambda *args, **kwargs: types.SimpleNamespace(
            call=lambda request: types.SimpleNamespace(state=True, errormsg="")
        ),
        Duration=lambda secs: secs,
        Time=lambda *args, **kwargs: 0.0,
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
        init_node=lambda *args, **kwargs: None,
    )
    fake_smach = _make_module("smach", State=FakeState, StateMachine=FakeStateMachine)
    fake_smach_ros = _make_module(
        "smach_ros",
        IntrospectionServer=lambda *args, **kwargs: types.SimpleNamespace(
            start=lambda: None,
            stop=lambda: None,
        ),
    )
    fake_actionlib = _make_module("actionlib", SimpleActionClient=FakeActionClient)
    fake_tf2_ros = _make_module(
        "tf2_ros",
        Buffer=lambda: types.SimpleNamespace(),
        TransformListener=lambda buffer: types.SimpleNamespace(buffer=buffer),
    )
    fake_geometry_msgs_msg = _make_module("geometry_msgs.msg", PoseStamped=DummyPoseStamped)
    fake_geometry_msgs = _make_module("geometry_msgs", msg=fake_geometry_msgs_msg)
    fake_restaurant_msgs_msg = _make_module(
        "seu_restaurant_msgs.msg",
        AlignAndTakeOrderAction=object,
        AlignAndTakeOrderGoal=lambda **kwargs: types.SimpleNamespace(**kwargs),
        FindWavingCustomerAction=object,
        FindWavingCustomerGoal=lambda **kwargs: types.SimpleNamespace(**kwargs),
        RestaurantNavAction=object,
        RestaurantNavGoal=DummyNavGoal,
    )
    fake_restaurant_msgs = _make_module("seu_restaurant_msgs", msg=fake_restaurant_msgs_msg)
    fake_voice_srv = _make_module(
        "seu_voice.srv",
        seu_voice=object,
        seu_voiceRequest=lambda text="": types.SimpleNamespace(text=text),
    )
    fake_voice = _make_module("seu_voice", srv=fake_voice_srv)
    fake_restaurant_states = _make_module(
        "restaurant_states",
        build_customer_nav_pose=lambda *args, **kwargs: None,
        reverse_pose_orientation=lambda pose: pose,
    )

    spec = importlib.util.spec_from_file_location("restaurant_smach_under_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    with patch.dict(
        sys.modules,
        {
            "rospy": fake_rospy,
            "smach": fake_smach,
            "smach_ros": fake_smach_ros,
            "actionlib": fake_actionlib,
            "tf2_ros": fake_tf2_ros,
            "geometry_msgs": fake_geometry_msgs,
            "geometry_msgs.msg": fake_geometry_msgs_msg,
            "seu_restaurant_msgs": fake_restaurant_msgs,
            "seu_restaurant_msgs.msg": fake_restaurant_msgs_msg,
            "seu_voice": fake_voice,
            "seu_voice.srv": fake_voice_srv,
            "restaurant_states": fake_restaurant_states,
        },
    ):
        spec.loader.exec_module(module)
    return module


def make_point(x, y, z=0.0):
    return types.SimpleNamespace(x=x, y=y, z=z)


class TwoCustomerFlowStateTest(unittest.TestCase):
    """双人调度相关状态测试。"""

    def test_build_bar_order_repeat_text_should_use_fixed_sentence_and_digit_words(self):
        module = load_module()

        repeat_text = module.build_bar_order_repeat_text(["cola", "chips"], [3, 1])

        self.assertEqual(
            repeat_text,
            "The customer ordered three cola, one chips, please give me them.",
        )

    def test_speak_order_at_bar_state_should_use_orderlist_instead_of_original_order_text(self):
        module = load_module()
        userdata = types.SimpleNamespace(
            order_text="I would like three bottles of cola and one bag of chips.",
            orderlist=["cola", "chips"],
            orderlist_num=[3, 1],
        )

        with patch.object(module, "say_once") as say_once_mock:
            outcome = module.SpeakOrderAtBarState().execute(userdata)

        self.assertEqual(outcome, "success")
        say_once_mock.assert_called_once_with(
            "The customer ordered three cola, one chips, please give me them."
        )

    def test_grasp_first_ordered_item_should_use_closed_set_for_matching_item(self):
        module = load_module()
        module.DEBUG = False
        module.CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays"]
        runtime = types.SimpleNamespace(name="runtime")
        grasp_result = types.SimpleNamespace(
            success=True,
            status="success",
            candidate_index=0,
            message="",
        )
        userdata = types.SimpleNamespace(orderlist=["Lays."], orderlist_num=[1])

        with patch.object(module, "get_grasp_runtime", return_value=runtime), patch.object(
            module,
            "_run_closed_set_bottle_grasp",
            return_value=grasp_result,
        ) as closed_set_mock, patch.object(module, "_GRASP_BY_CATEGORY") as grasp_by_category_mock:
            outcome = module.GraspFirstOrderedItemState().execute(userdata)

        self.assertEqual(outcome, "success")
        closed_set_mock.assert_called_once_with(
            runtime,
            category="bottle",
            closed_set_gdino_prompt="bottle.lays.",
            target_prompt="lays",
        )
        grasp_by_category_mock.assert_not_called()

    def test_grasp_first_ordered_item_should_fallback_to_grasp_by_category_for_unknown_item(self):
        module = load_module()
        module.DEBUG = False
        module.CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays"]
        runtime = types.SimpleNamespace(name="runtime")
        grasp_result = types.SimpleNamespace(
            success=True,
            status="success",
            candidate_index=0,
            message="",
        )
        userdata = types.SimpleNamespace(orderlist=["Chips"], orderlist_num=[1])

        with patch.object(module, "get_grasp_runtime", return_value=runtime), patch.object(
            module,
            "_run_closed_set_bottle_grasp",
        ) as closed_set_mock, patch.object(
            module,
            "_GRASP_BY_CATEGORY",
            return_value=grasp_result,
        ) as grasp_by_category_mock:
            outcome = module.GraspFirstOrderedItemState().execute(userdata)

        self.assertEqual(outcome, "success")
        grasp_by_category_mock.assert_called_once_with(runtime, "bottle", bottle_prompt="chips")
        closed_set_mock.assert_not_called()

    def test_find_waving_customers_state_should_store_multiple_points(self):
        module = load_module()
        FakeActionClient.next_result = types.SimpleNamespace(
            success=True,
            person_points_map=[make_point(1.0, 2.0), make_point(3.0, 4.0)],
        )
        userdata = types.SimpleNamespace()

        outcome = module.FindWavingCustomersState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(len(userdata.detected_customer_points), 2)
        self.assertAlmostEqual(userdata.detected_customer_points[1].x, 3.0)

    def test_load_customer_queue_should_trim_to_remaining_count(self):
        module = load_module()
        userdata = types.SimpleNamespace(
            detected_customer_points=[make_point(1.0, 0.0), make_point(2.0, 0.0)],
            served_customer_count=1,
        )

        outcome = module.LoadCustomerQueueState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(len(userdata.pending_customer_points), 1)
        self.assertAlmostEqual(userdata.pending_customer_points[0].x, 1.0)

    def test_select_next_customer_should_pop_first_pending_customer(self):
        module = load_module()
        userdata = types.SimpleNamespace(
            pending_customer_points=[make_point(1.0, 0.0), make_point(2.0, 0.0)],
        )

        outcome = module.SelectNextCustomerState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertAlmostEqual(userdata.customer_point_map.x, 1.0)
        self.assertEqual(len(userdata.pending_customer_points), 1)
        self.assertAlmostEqual(userdata.pending_customer_points[0].x, 2.0)

    def test_navigate_back_to_start_pose_should_reverse_bar_pose_inside_state(self):
        module = load_module()
        FakeActionClient.next_result = types.SimpleNamespace(success=True)
        bar_pose = types.SimpleNamespace(name="bar_pose")
        start_pose = types.SimpleNamespace(name="start_pose")
        userdata = types.SimpleNamespace(bar_pose_map=bar_pose)

        with patch.object(module, "reverse_pose_orientation", return_value=start_pose) as reverse_mock:
            outcome = module.NavigateBackToStartPoseState().execute(userdata)

        self.assertEqual(outcome, "success")
        reverse_mock.assert_called_once_with(bar_pose)
        self.assertEqual(len(FakeActionClient.instances), 1)
        sent_goal = FakeActionClient.instances[0].goals[0]
        self.assertIs(sent_goal.nav_pose, start_pose)

    def test_mark_customer_served_and_decide_should_serve_pending_before_rescan(self):
        module = load_module()
        userdata = types.SimpleNamespace(
            served_customer_count=0,
            pending_customer_points=[make_point(2.0, 0.0)],
        )

        mark_outcome = module.MarkCustomerServedState().execute(userdata)
        decide_outcome = module.DecideNextStepState().execute(userdata)

        self.assertEqual(mark_outcome, "success")
        self.assertEqual(userdata.served_customer_count, 1)
        self.assertEqual(decide_outcome, "serve_next_pending")

    def test_decide_next_step_should_request_rescan_when_queue_empty_and_not_finished(self):
        module = load_module()
        userdata = types.SimpleNamespace(
            served_customer_count=1,
            pending_customer_points=[],
        )

        outcome = module.DecideNextStepState().execute(userdata)

        self.assertEqual(outcome, "find_more")

    def test_decide_next_step_should_finish_after_two_customers(self):
        module = load_module()
        userdata = types.SimpleNamespace(
            served_customer_count=2,
            pending_customer_points=[make_point(3.0, 0.0)],
        )

        outcome = module.DecideNextStepState().execute(userdata)

        self.assertEqual(outcome, "finished")


if __name__ == "__main__":
    unittest.main()
