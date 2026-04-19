#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试双人服务调度相关的状态逻辑。"""

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import call, patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "restaurant_smach.py"


class DummyPoseStamped:
    pass


class DummyTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class DummyNavGoal:
    TARGET_MODE_POSE = 2
    TARGET_MODE_NAME = 1

    def __init__(self):
        self.target_mode = None
        self.nav_pose = None
        self.pid_pose = None
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

    def fake_time(*args, **kwargs):
        del args, kwargs
        return 0.0

    fake_time.now = lambda: 0.0

    fake_rospy = _make_module(
        "rospy",
        wait_for_service=lambda *args, **kwargs: None,
        Publisher=lambda *args, **kwargs: types.SimpleNamespace(publish=lambda msg: None),
        Rate=lambda hz: types.SimpleNamespace(sleep=lambda: None),
        ServiceProxy=lambda *args, **kwargs: types.SimpleNamespace(
            call=lambda request: types.SimpleNamespace(state=True, errormsg="")
        ),
        Duration=lambda secs: secs,
        Time=fake_time,
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
        init_node=lambda *args, **kwargs: None,
        sleep=lambda *args, **kwargs: None,
        is_shutdown=lambda: False,
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
    fake_tf_transformations = _make_module(
        "tf.transformations",
        euler_from_quaternion=lambda quaternion: (0.0, 0.0, 0.0),
    )
    fake_tf = _make_module("tf", transformations=fake_tf_transformations)
    fake_geometry_msgs_msg = _make_module(
        "geometry_msgs.msg",
        PoseStamped=DummyPoseStamped,
        Twist=DummyTwist,
    )
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
        build_bar_nav_and_pid_poses=lambda *args, **kwargs: (None, None),
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
            "tf": fake_tf,
            "tf.transformations": fake_tf_transformations,
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

    def test_hot_start_grasp_runtime_should_wait_for_moveit_services_before_loading_runtime(self):
        module = load_module()
        runtime = types.SimpleNamespace(name="runtime")

        with patch.object(module.rospy, "wait_for_service") as wait_for_service_mock, patch.object(
            module,
            "get_grasp_runtime",
            return_value=runtime,
        ) as get_grasp_runtime_mock:
            result = module.hot_start_grasp_runtime()

        self.assertIs(result, runtime)
        self.assertEqual(
            wait_for_service_mock.call_args_list,
            [
                call("joint_moveit_ctrl_arm", timeout=30.0),
                call("joint_moveit_ctrl_gripper", timeout=30.0),
            ],
        )
        get_grasp_runtime_mock.assert_called_once_with()

    def test_main_should_hot_start_grasp_runtime_before_building_state_machine(self):
        module = load_module()
        event_log = []
        fake_sm = types.SimpleNamespace(
            execute=lambda: event_log.append("execute") or "task_success",
        )
        fake_introspection_server = types.SimpleNamespace(
            start=lambda: event_log.append("introspection_start"),
            stop=lambda: event_log.append("introspection_stop"),
        )

        with patch.object(
            module.rospy,
            "init_node",
            side_effect=lambda *args, **kwargs: event_log.append("init_node"),
        ), patch.object(
            module,
            "hot_start_grasp_runtime",
            side_effect=lambda: event_log.append("hot_start"),
        ), patch.object(
            module,
            "build_top_level_state_machine",
            side_effect=lambda: event_log.append("build_sm") or fake_sm,
        ), patch.object(
            module.smach_ros,
            "IntrospectionServer",
            side_effect=lambda *args, **kwargs: event_log.append("make_introspection_server")
            or fake_introspection_server,
        ), patch.object(
            module,
            "shutdown_grasp_runtime",
            side_effect=lambda: event_log.append("shutdown_grasp_runtime"),
        ):
            module.main()

        self.assertEqual(
            event_log,
            [
                "init_node",
                "hot_start",
                "build_sm",
                "make_introspection_server",
                "introspection_start",
                "execute",
                "shutdown_grasp_runtime",
                "introspection_stop",
            ],
        )

    def test_grasp_first_ordered_item_should_use_closed_set_for_matching_item(self):
        module = load_module()
        module.DEBUG = False
        module.CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays"]
        runtime = types.SimpleNamespace(name="runtime")
        initial_pose = types.SimpleNamespace(name="initial_pose")
        initial_yaw = 0.3
        grasp_result = types.SimpleNamespace(
            success=True,
            status="success",
            candidate_index=0,
            message="",
        )
        userdata = types.SimpleNamespace(orderlist=["Lays."], orderlist_num=[1])

        with patch.object(module, "lookup_robot_pose", return_value=initial_pose) as lookup_mock, patch.object(
            module,
            "pose_to_yaw",
            return_value=initial_yaw,
        ), patch.object(module, "rotate_base_to_relative_yaw") as rotate_mock, patch.object(
            module,
            "move_base_to_reference_forward_offset",
            return_value=True,
        ) as linear_move_mock, patch.object(
            module,
            "get_grasp_runtime",
            return_value=runtime,
        ), patch.object(
            module,
            "_GRASP_BOTTLE_CLOSED_SET",
            return_value=grasp_result,
        ) as closed_set_mock, patch.object(
            module,
            "drive_base_backward_after_grasp",
            return_value=True,
        ), patch.object(module, "_GRASP_BOTTLE") as grasp_by_category_mock:
            outcome = module.GraspFirstOrderedItemState().execute(userdata)

        self.assertEqual(outcome, "success")
        lookup_mock.assert_called_once_with(timeout_sec=2.0)
        closed_set_mock.assert_called_once_with(
            runtime,
            target_prompt="lays",
            gdino_prompt="cola.lays.",
        )
        rotate_mock.assert_called_once_with(initial_yaw, 0.0)
        linear_move_mock.assert_called_once_with(
            initial_pose,
            0.0,
            linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
            tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
            control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
            stage_label="grasp_retry_restore",
        )
        grasp_by_category_mock.assert_not_called()

    def test_grasp_first_ordered_item_should_rotate_for_no_match_then_retry(self):
        module = load_module()
        module.DEBUG = False
        module.CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays"]
        runtime = types.SimpleNamespace(name="runtime")
        initial_pose = types.SimpleNamespace(name="initial_pose")
        initial_yaw = -0.4
        grasp_result_list = [
            types.SimpleNamespace(
                success=False,
                status="failed",
                candidate_index=-1,
                message="",
                detect_result={"status": "closed_set_no_match"},
            ),
            types.SimpleNamespace(
                success=True,
                status="success",
                candidate_index=0,
                message="",
            ),
        ]
        userdata = types.SimpleNamespace(orderlist=["Cola."], orderlist_num=[1])

        with patch.object(module, "lookup_robot_pose", return_value=initial_pose), patch.object(
            module,
            "pose_to_yaw",
            return_value=initial_yaw,
        ), patch.object(module, "rotate_base_to_relative_yaw") as rotate_mock, patch.object(
            module,
            "move_base_to_reference_forward_offset",
            return_value=True,
        ) as linear_move_mock, patch.object(
            module,
            "get_grasp_runtime",
            return_value=runtime,
        ), patch.object(
            module,
            "_GRASP_BOTTLE_CLOSED_SET",
            side_effect=grasp_result_list,
        ) as closed_set_mock, patch.object(
            module,
            "drive_base_backward_after_grasp",
            return_value=True,
        ), patch.object(module, "_GRASP_BOTTLE") as grasp_by_category_mock:
            outcome = module.GraspFirstOrderedItemState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(
            closed_set_mock.call_args_list,
            [
                call(runtime, target_prompt="cola", gdino_prompt="cola.lays."),
                call(runtime, target_prompt="cola", gdino_prompt="cola.lays."),
            ],
        )
        self.assertEqual(
            rotate_mock.call_args_list,
            [
                call(initial_yaw, math.radians(15.0)),
                call(initial_yaw, 0.0),
            ],
        )
        linear_move_mock.assert_called_once_with(
            initial_pose,
            0.0,
            linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
            tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
            control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
            stage_label="grasp_retry_restore",
        )
        grasp_by_category_mock.assert_not_called()

    def test_grasp_first_ordered_item_should_rotate_to_negative_15_degree_for_third_no_match_retry(self):
        module = load_module()
        module.DEBUG = False
        module.CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays"]
        runtime = types.SimpleNamespace(name="runtime")
        initial_pose = types.SimpleNamespace(name="initial_pose")
        initial_yaw = 1.2
        userdata = types.SimpleNamespace(orderlist=["Cola"], orderlist_num=[1])

        with patch.object(module, "lookup_robot_pose", return_value=initial_pose), patch.object(
            module,
            "pose_to_yaw",
            return_value=initial_yaw,
        ), patch.object(module, "rotate_base_to_relative_yaw") as rotate_mock, patch.object(
            module,
            "move_base_to_reference_forward_offset",
            return_value=True,
        ) as linear_move_mock, patch.object(
            module,
            "get_grasp_runtime",
            return_value=runtime,
        ), patch.object(
            module,
            "_GRASP_BOTTLE_CLOSED_SET",
            side_effect=[
                types.SimpleNamespace(
                    success=False,
                    status="failed",
                    candidate_index=-1,
                    message="",
                    detect_result={"status": "closed_set_no_match"},
                ),
                types.SimpleNamespace(
                    success=False,
                    status="failed",
                    candidate_index=-1,
                    message="",
                    detect_result={"status": "closed_set_no_match"},
                ),
                types.SimpleNamespace(success=True, status="success", candidate_index=0, message=""),
            ],
        ) as closed_set_mock, patch.object(
            module,
            "drive_base_backward_after_grasp",
            return_value=True,
        ), patch.object(
            module,
            "_GRASP_BOTTLE",
        ) as grasp_by_category_mock:
            outcome = module.GraspFirstOrderedItemState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(
            closed_set_mock.call_args_list,
            [
                call(runtime, target_prompt="cola", gdino_prompt="cola.lays."),
                call(runtime, target_prompt="cola", gdino_prompt="cola.lays."),
                call(runtime, target_prompt="cola", gdino_prompt="cola.lays."),
            ],
        )
        self.assertEqual(
            rotate_mock.call_args_list,
            [
                call(initial_yaw, math.radians(15.0)),
                call(initial_yaw, math.radians(-15.0)),
                call(initial_yaw, 0.0),
            ],
        )
        linear_move_mock.assert_called_once_with(
            initial_pose,
            0.0,
            linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
            tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
            control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
            stage_label="grasp_retry_restore",
        )
        grasp_by_category_mock.assert_not_called()

    def test_grasp_first_ordered_item_should_use_linear_retries_for_grasp_errors(self):
        module = load_module()
        module.DEBUG = False
        module.CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays", "water"]
        runtime = types.SimpleNamespace(name="runtime")
        initial_pose_list = [
            types.SimpleNamespace(name="pose_1"),
            types.SimpleNamespace(name="pose_2"),
            types.SimpleNamespace(name="pose_3"),
        ]
        initial_yaw_list = [0.1, 0.2, 0.3]
        userdata = types.SimpleNamespace(
            orderlist=["Cola", "Lays", "Water", "Milk"],
            orderlist_num=[1, 1, 1, 1],
        )

        with patch.object(
            module,
            "lookup_robot_pose",
            side_effect=initial_pose_list,
        ) as lookup_mock, patch.object(
            module,
            "pose_to_yaw",
            side_effect=initial_yaw_list,
        ), patch.object(module, "rotate_base_to_relative_yaw") as rotate_mock, patch.object(
            module,
            "move_base_to_reference_forward_offset",
            return_value=True,
        ) as linear_move_mock, patch.object(
            module,
            "get_grasp_runtime",
            return_value=runtime,
        ), patch.object(
            module,
            "_GRASP_BOTTLE_CLOSED_SET",
            side_effect=[
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
                types.SimpleNamespace(success=False, status="failed", candidate_index=-1, message=""),
            ],
        ) as closed_set_mock, patch.object(
            module,
            "drive_base_backward_after_grasp",
            return_value=True,
        ):
            outcome = module.GraspFirstOrderedItemState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(lookup_mock.call_count, 3)
        self.assertEqual(closed_set_mock.call_count, 9)
        self.assertEqual(
            [call_args.kwargs["target_prompt"] for call_args in closed_set_mock.call_args_list],
            ["cola", "cola", "cola", "lays", "lays", "lays", "water", "water", "water"],
        )
        self.assertEqual(
            rotate_mock.call_args_list,
            [
                call(0.1, 0.0),
                call(0.2, 0.0),
                call(0.3, 0.0),
            ],
        )
        self.assertEqual(
            linear_move_mock.call_args_list,
            [
                call(
                    initial_pose_list[0],
                    -0.15,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_linear",
                ),
                call(
                    initial_pose_list[0],
                    0.05,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_linear",
                ),
                call(
                    initial_pose_list[0],
                    0.0,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_restore",
                ),
                call(
                    initial_pose_list[1],
                    -0.15,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_linear",
                ),
                call(
                    initial_pose_list[1],
                    0.05,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_linear",
                ),
                call(
                    initial_pose_list[1],
                    0.0,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_restore",
                ),
                call(
                    initial_pose_list[2],
                    -0.15,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_linear",
                ),
                call(
                    initial_pose_list[2],
                    0.05,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_linear",
                ),
                call(
                    initial_pose_list[2],
                    0.0,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_restore",
                ),
            ],
        )

    def test_grasp_first_ordered_item_should_use_linear_retry_for_grasp_exception(self):
        module = load_module()
        module.DEBUG = False
        module.CLOSED_SET_ITEM_NAME_LIST = ["cola", "lays"]
        runtime = types.SimpleNamespace(name="runtime")
        initial_pose = types.SimpleNamespace(name="initial_pose")
        initial_yaw = -0.8
        userdata = types.SimpleNamespace(orderlist=["Cola"], orderlist_num=[1])
        grasp_result = types.SimpleNamespace(
            success=True,
            status="success",
            candidate_index=0,
            message="",
        )

        with patch.object(module, "lookup_robot_pose", return_value=initial_pose), patch.object(
            module,
            "pose_to_yaw",
            return_value=initial_yaw,
        ), patch.object(
            module,
            "rotate_base_to_relative_yaw",
        ) as rotate_mock, patch.object(
            module,
            "move_base_to_reference_forward_offset",
            return_value=True,
        ) as linear_move_mock, patch.object(
            module,
            "get_grasp_runtime",
            return_value=runtime,
        ), patch.object(
            module,
            "_GRASP_BOTTLE_CLOSED_SET",
            side_effect=[RuntimeError("detect failed"), grasp_result],
        ) as closed_set_mock, patch.object(
            module,
            "drive_base_backward_after_grasp",
            return_value=True,
        ):
            outcome = module.GraspFirstOrderedItemState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(
            closed_set_mock.call_args_list,
            [
                call(runtime, target_prompt="cola", gdino_prompt="cola.lays."),
                call(runtime, target_prompt="cola", gdino_prompt="cola.lays."),
            ],
        )
        self.assertEqual(
            rotate_mock.call_args_list,
            [
                call(initial_yaw, 0.0),
            ],
        )
        self.assertEqual(
            linear_move_mock.call_args_list,
            [
                call(
                    initial_pose,
                    -0.15,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_linear",
                ),
                call(
                    initial_pose,
                    0.0,
                    linear_speed_m_s=module.GRASP_RETRY_LINEAR_SPEED_M_S,
                    tolerance_m=module.GRASP_RETRY_LINEAR_TOLERANCE_M,
                    control_hz=module.GRASP_RETRY_LINEAR_CONTROL_HZ,
                    stage_label="grasp_retry_restore",
                ),
            ],
        )

    def test_drive_base_backward_after_grasp_should_use_tf_closed_loop_helper(self):
        module = load_module()
        reference_pose = types.SimpleNamespace(name="reference_pose")

        with patch.object(module, "lookup_robot_pose", return_value=reference_pose) as lookup_mock, patch.object(
            module,
            "move_base_to_reference_forward_offset",
            return_value=True,
        ) as move_mock:
            result = module.drive_base_backward_after_grasp()

        self.assertTrue(result)
        lookup_mock.assert_called_once_with(timeout_sec=2.0)
        move_mock.assert_called_once_with(
            reference_pose,
            -float(module.GRASP_BACKOFF_DISTANCE_M),
            linear_speed_m_s=module.GRASP_BACKOFF_SPEED_M_S,
            tolerance_m=module.GRASP_BACKOFF_TOLERANCE_M,
            control_hz=module.GRASP_BACKOFF_CONTROL_HZ,
            stage_label="grasp_backoff",
        )

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

    def test_navigate_pose_state_should_forward_pid_pose_when_explicitly_configured(self):
        module = load_module()
        FakeActionClient.next_result = types.SimpleNamespace(success=True)
        nav_pose = types.SimpleNamespace(name="nav_pose")
        pid_pose = types.SimpleNamespace(name="pid_pose")
        userdata = types.SimpleNamespace(bar_pose_map=nav_pose, bar_pid_pose_map=pid_pose)

        outcome = module.NavigatePoseState("bar_pose_map", pid_pose_key="bar_pid_pose_map").execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(len(FakeActionClient.instances), 1)
        sent_goal = FakeActionClient.instances[0].goals[0]
        self.assertIs(sent_goal.nav_pose, nav_pose)
        self.assertIs(sent_goal.pid_pose, pid_pose)

    def test_navigate_back_to_start_pose_should_send_final_bar_nav_pose_only(self):
        module = load_module()
        FakeActionClient.next_result = types.SimpleNamespace(success=True)
        final_nav_pose = types.SimpleNamespace(name="final_nav_pose")
        userdata = types.SimpleNamespace(
            bar_final_nav_pose_map=final_nav_pose,
        )

        outcome = module.NavigateBackToStartPoseState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertEqual(len(FakeActionClient.instances), 1)
        sent_goal = FakeActionClient.instances[0].goals[0]
        self.assertIs(sent_goal.nav_pose, final_nav_pose)
        self.assertIsNone(sent_goal.pid_pose)

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
