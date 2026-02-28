#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
模拟外部话题发布器
==================
该脚本发布 person_analyse 服务需要的外部 ROS 话题，
用于在没有真实硬件或其他节点的情况下测试服务。

使用方法:
1. 启动 roscore
2. 运行此脚本: python mock_topics.py
3. 在另一个终端运行服务: rosrun seu_person_analyse person_analyse.py
4. 调用服务: rosservice call /person_analyse_service "feature: 'all'"

发布的话题:
- /robot_pose (geometry_msgs/PoseStamped) - 模拟机器人位姿
"""

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
import time

class MockTopicPublisher:
    """模拟外部话题发布器"""
    
    def __init__(self):
        rospy.init_node('mock_topic_publisher', anonymous=True)
        
        # 发布机器人位姿
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        
        # 订阅速度命令（用于调试）
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 模拟位姿数据
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        rospy.loginfo("Mock Topic Publisher 启动")
        rospy.loginfo("发布话题: /robot_pose")
        rospy.loginfo("订阅话题: /cmd_vel")
    
    def cmd_vel_callback(self, msg):
        """处理速度命令，更新模拟位姿"""
        dt = 0.1  # 假设 10Hz
        
        # 更新位姿
        self.x += msg.linear.x * math.cos(self.theta) * dt
        self.y += msg.linear.x * math.sin(self.theta) * dt
        self.theta += msg.angular.z * dt
        
        # 归一化角度
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        rospy.logdebug(f"更新位姿: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
    
    def publish_pose(self):
        """发布当前位姿"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        
        # 从 theta 计算四元数 (简化：只绕 z 轴旋转)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.theta / 2)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2)
        
        self.pose_pub.publish(pose_msg)
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo("开始发布模拟位姿...")
        
        while not rospy.is_shutdown():
            self.publish_pose()
            rate.sleep()

def main():
    try:
        publisher = MockTopicPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
