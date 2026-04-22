#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试启动时记录吧台位姿的状态。"""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "restaurant_smach.py"


class DummyPoseStamped:
    pass


class DummyTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class DummyPointCloud2:
    pass


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
    def __init__(self, *args, **kwargs):
        pass


def _make_module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def load_module():
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
    fake_geometry_msgs_msg = _make_module(
        "geometry_msgs.msg",
        PoseStamped=DummyPoseStamped,
        Twist=DummyTwist,
    )
    fake_geometry_msgs = _make_module("geometry_msgs", msg=fake_geometry_msgs_msg)
    fake_sensor_msgs_msg = _make_module("sensor_msgs.msg", PointCloud2=DummyPointCloud2)
    fake_sensor_msgs_point_cloud2 = _make_module(
        "sensor_msgs.point_cloud2",
        read_points=lambda *args, **kwargs: iter(()),
    )
    fake_sensor_msgs = _make_module(
        "sensor_msgs",
        msg=fake_sensor_msgs_msg,
        point_cloud2=fake_sensor_msgs_point_cloud2,
    )
    fake_restaurant_msgs_msg = _make_module(
        "seu_restaurant_msgs.msg",
        AlignAndTakeOrderAction=object,
        AlignAndTakeOrderGoal=lambda **kwargs: kwargs,
        FindWavingCustomerAction=object,
        FindWavingCustomerGoal=lambda **kwargs: kwargs,
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
            "tf2_ros": fake_tf2_ros,
            "geometry_msgs": fake_geometry_msgs,
            "geometry_msgs.msg": fake_geometry_msgs_msg,
            "sensor_msgs": fake_sensor_msgs,
            "sensor_msgs.msg": fake_sensor_msgs_msg,
            "sensor_msgs.point_cloud2": fake_sensor_msgs_point_cloud2,
            "seu_restaurant_msgs": fake_restaurant_msgs,
            "seu_restaurant_msgs.msg": fake_restaurant_msgs_msg,
            "seu_voice": fake_voice,
            "seu_voice.srv": fake_voice_srv,
            "restaurant_states": fake_restaurant_states,
        },
    ):
        spec.loader.exec_module(module)
    return module


class CaptureBarPoseStateTest(unittest.TestCase):
    """启动位姿捕获状态测试。"""

    def test_capture_bar_pose_state_reads_pose_and_saves_grasp_and_final_bar_targets(self):
        module = load_module()
        current_pose = types.SimpleNamespace(name="current_pose")
        bar_nav_pose = types.SimpleNamespace(name="bar_nav_pose")
        bar_pid_pose = types.SimpleNamespace(name="bar_pid_pose")
        bar_final_nav_pose = types.SimpleNamespace(name="bar_final_nav_pose")
        bar_final_pid_pose = types.SimpleNamespace(name="bar_final_pid_pose")
        userdata = types.SimpleNamespace()

        with patch.object(module, "lookup_robot_pose", return_value=current_pose) as lookup_mock, patch.object(
            module,
            "build_bar_nav_and_pid_poses",
            side_effect=[
                (bar_nav_pose, bar_pid_pose),
                (bar_final_nav_pose, bar_final_pid_pose),
            ],
        ) as build_bar_mock:
            outcome = module.CaptureBarPoseState().execute(userdata)

        self.assertEqual(outcome, "success")
        self.assertIs(userdata.bar_pose_map, bar_nav_pose)
        self.assertIs(userdata.bar_pid_pose_map, bar_pid_pose)
        self.assertIs(userdata.bar_final_nav_pose_map, bar_final_nav_pose)
        self.assertIs(userdata.bar_final_pid_pose_map, bar_final_pid_pose)
        lookup_mock.assert_called_once_with(timeout_sec=2.0)
        self.assertEqual(build_bar_mock.call_count, 2)
        self.assertEqual(
            build_bar_mock.call_args_list[0].args,
            (current_pose, module.BAR_NAV_EXTENSION_DISTANCE_M),
        )
        self.assertEqual(
            build_bar_mock.call_args_list[1].args,
            (current_pose, module.BAR_NAV_EXTENSION_DISTANCE_M),
        )
        self.assertEqual(
            build_bar_mock.call_args_list[1].kwargs,
            {"pid_reverse_orientation": False, "nav_reverse_orientation": False},
        )


if __name__ == "__main__":
    unittest.main()
