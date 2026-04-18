#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy
import smach

import restaurant_smach


TARGET_ITEM_NAME = "lays"
TARGET_ITEM_COUNT = 1


def coerce_closed_set_item_names(value):
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item).strip() for item in list(value or []) if str(item).strip()]


def main():
    rospy.init_node("test_grasp_first_ordered_item_state")

    target_item_name = rospy.get_param("~target_item_name", TARGET_ITEM_NAME)
    target_item_count = rospy.get_param("~target_item_count", TARGET_ITEM_COUNT)
    debug = bool(rospy.get_param("~debug", False))
    closed_set_item_names = coerce_closed_set_item_names(
        rospy.get_param("~closed_set_item_names", restaurant_smach.CLOSED_SET_ITEM_NAME_LIST)
    )

    restaurant_smach.DEBUG = debug
    restaurant_smach.CLOSED_SET_ITEM_NAME_LIST = closed_set_item_names

    userdata = smach.UserData()
    userdata.orderlist = [target_item_name]
    userdata.orderlist_num = [target_item_count]

    state = restaurant_smach.GraspFirstOrderedItemState()
    normalized_item_name = restaurant_smach.normalize_item_name_for_closed_set(target_item_name)
    closed_set_name_set = restaurant_smach.build_closed_set_item_name_set()
    gdino_prompt = restaurant_smach.build_closed_set_gdino_prompt()

    rospy.loginfo(
        "Start grasp state test | item=%s normalized_item=%s count=%s debug=%s closed_set_items=%s gdino_prompt=%s",
        target_item_name,
        normalized_item_name,
        target_item_count,
        debug,
        sorted(closed_set_name_set),
        gdino_prompt,
    )

    try:
        outcome = state.execute(userdata)
        rospy.loginfo("Grasp state test finished with outcome: %s", outcome)
        sys.exit(0 if outcome == "success" else 1)
    finally:
        restaurant_smach.shutdown_grasp_runtime()


if __name__ == "__main__":
    main()
