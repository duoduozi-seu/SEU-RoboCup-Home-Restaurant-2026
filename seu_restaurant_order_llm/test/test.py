#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订单解析 action client 测试脚本。"""

import actionlib
import rospy

from seu_restaurant_order_llm.msg import ParseRestaurantOrderAction, ParseRestaurantOrderGoal


# 这里直接写一个固定英文句子，方便独立联调。
TEST_ORDER_TEXT = "i want a bottle of cola, a bag of chips and a bottle of mineral water"
ACTION_NAME = "parse_restaurant_order"


def main():
    """发送一个测试 goal，并把服务端返回结果打印出来。"""
    # 先初始化 ROS 节点，保证 action client 能正常注册到 ROS 图里。
    rospy.init_node("restaurant_order_llm_test_client")

    # 这里直接连新的订单解析 action server。
    client = actionlib.SimpleActionClient(ACTION_NAME, ParseRestaurantOrderAction)
    rospy.loginfo("[restaurant_order_llm_test] waiting for action server: %s", ACTION_NAME)
    client.wait_for_server()

    # 目标内容只有一句订单文本，保持接口最小化。
    goal = ParseRestaurantOrderGoal()
    goal.order_text = TEST_ORDER_TEXT
    client.send_goal(goal)

    # 等待服务端完成本次解析，再统一读取结果。
    client.wait_for_result()
    result = client.get_result()

    print("success:", result.success)
    print("error_msg:", result.error_msg)
    print("items:", list(result.items))


if __name__ == "__main__":
    main()
