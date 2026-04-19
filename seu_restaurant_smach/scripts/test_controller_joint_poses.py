#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy

import restaurant_smach


TEST_JOINT_POSES = [
    ("pose_1", [1.605, 1.386, -0.908, 0.00, 1.191, -1.538]),
    ("pose_2", [1.605, 1.703, -0.592, 0.0, 0.559, -1.538]),
    ("pose_3", [1.605, 1.386, -0.908, 0.00, 1.191, -1.538]),
    ("pose_4", [0.0, 1.491, -0.942, 0.0, -0.549, 0.0]),
]
DEFAULT_STAY_SEC = 1.0


def main():
    rospy.init_node("test_controller_joint_poses")

    stay_sec = float(rospy.get_param("~stay_sec", DEFAULT_STAY_SEC))
    controller = restaurant_smach.get_piper_deliver_controller()
    # controller.set_gripper_width_with_effort(start_width=0.098)
    restaurant_smach.release_gripper_once()
    # rospy.sleep(1)
    # controller.set_gripper_width_with_effort(start_width=0.098)
    rospy.loginfo("Start joint pose controller test | stay_sec=%.1f", stay_sec)
    for pose_name, joints in TEST_JOINT_POSES:
        if rospy.is_shutdown():
            break
        # if pose_name == "pose_2":
            # break
        
        restaurant_smach.move_arm_to_joint_positions_once(
            joints,
            description=pose_name,
            controller=controller,
        )
        if pose_name == "pose_2":
          controller.set_gripper_width_with_effort(start_width=0.098)
          # restaurant_smach.release_gripper_once()
          # break
        
        rospy.loginfo("Reached %s, hold for %.1f seconds", pose_name, stay_sec)
        rospy.sleep(stay_sec)

    rospy.loginfo("Joint pose controller test finished.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
    except Exception as exc:
        rospy.logerr("Joint pose controller test failed: %s", str(exc))
        sys.exit(1)
