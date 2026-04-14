#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy
import smach

import restaurant_smach


TARGET_ITEM_NAME = "layss"
TARGET_ITEM_COUNT = 1


def main():
    rospy.init_node("test_grasp_first_ordered_item_state")

    # 关闭原状态里的调试短路，确保真正进入抓取调用。
    restaurant_smach.DEBUG = False

    userdata = smach.UserData()
    userdata.orderlist = [TARGET_ITEM_NAME]
    userdata.orderlist_num = [TARGET_ITEM_COUNT]

    state = restaurant_smach.GraspFirstOrderedItemState()

    rospy.loginfo(
        "Start grasp state test | item=%s count=%s",
        TARGET_ITEM_NAME,
        TARGET_ITEM_COUNT,
    )

    try:
        outcome = state.execute(userdata)
        rospy.loginfo("Grasp state test finished with outcome: %s", outcome)
        sys.exit(0 if outcome == "success" else 1)
    finally:
        restaurant_smach.shutdown_grasp_runtime()


if __name__ == "__main__":
    main()
