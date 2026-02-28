#! /home/syw/anaconda3/envs/GPSR/bin/python
# -*- coding: utf-8 -*-
import sys
# from pyexpat import features

import cv2
import time as time_module

import pyrealsense2 as rs
import numpy as np
from PIL import Image

sys.path.append("/home/syw/code/robocup_nation/2024code/GPSR/src")
from common.camera import *


# from camera import *
import rospy
from std_msgs.msg import String
from seu_robot_msgs.msg import object_position

import sensor_msgs.msg
from cv_bridge import CvBridge

camera_state = False
camera = Camera(serial_number = 0)

food_list = ['chip', 'biscuit', 'bread','cookie', 'lays']
drink_list = ['sprite', 'cola', 'water']
cleaning_stuff = ['dishsoap', 'handwash', 'shampoo']
def is_target_kind(kind,obj):
    if kind == 'food':
        return obj.lower() in food_list
    elif kind == 'drink':
        return obj.lower() in drink_list
    elif kind == 'cleaning stuff':
        return obj.lower() in cleaning_stuff
    elif kind.lower() in ['object','objects','any','good','goods','things','thing','item','items']:
        return True
    elif kind == obj:
        return True
    else:
        # 如果kind不是上述任何一种，可以返回False或抛出异常
        return False

def get_info(key):
    try:
        result = rospy.get_param(key)
    except:
        result = None
    return result

# 接收信号，决定是否开始检测
def CameraStateCallback(msg):
    global camera_state
    global camera_state_last

    camera_state_last = camera_state

    if msg.data == "camera_open":
        camera_state = True
    elif msg.data == "camera_close":
        camera_state = False

    if camera_state_last == False:
        if camera_state == True:
            #开启视频流
            camera.start_camera()
            # rospy.sleep(2)
            rospy.logwarn('Start the analysis of finding obj')

    elif camera_state_last == True:
        if camera_state == False:
            #关闭视频流
            camera.stop_camera()
            rospy.logwarn('Close the analysis of finding obj')

class ObjAnalyse:
    def __init__(self):
        self.obj_count = 0
        self.obj_count_last = 0
        self.obj_count_counted = 0
        self.count_list = {}
        self.pos_list = {}  # 用于存储位置结果
        self.pos_most_list = {}  # 用于存储最左/最右结果
        self.start_time = time_module.time()
        self.is_analyze = False
        self.obj_list = {}

    def update_time(self):
        if not self.is_analyze:
            print("update time",time_module.time())
            self.start_time = time_module.time()
            self.is_analyze = True
    def do_count(self,object,results):
        num = 0
        for res in results:
            detected_label = res['label']
            pos = res['pos']
            if pos[2] < 2 and  is_target_kind(object, detected_label):
                num = num + 1
        if num in self.count_list .keys():
            self.count_list[num] +=1
        else:
            self.count_list[num] = 1
        if time_module.time() - self.start_time>10:
                res = max(self.count_list, key=lambda k: self.count_list[k])
                self.obj_count = 0
                self.obj_count_last = 0
                self.obj_count_counted = 0
                self.count_list = {}
                return res

        # if self.obj_count == self.obj_count_last:
        #     self.obj_count_counted = self.obj_count_counted + 1
        #     if self.obj_count_counted > 5 and time_module.time() - self.start_time>10:
        #         res =  self.obj_count
        #         self.obj_count = 0
        #         self.obj_count_last = 0
        #         self.obj_count_counted = 0
        #         return res
        else:
            return None

    def do_analyse(self,object,results):
        for res in results:
            detected_label = res['label']
            pos = res['pos']
            if pos[2] < 2 and is_target_kind(object, detected_label):
                if detected_label in self.obj_list.keys():
                    self.obj_list[detected_label] = self.obj_list[detected_label] + 1
                else:
                    self.obj_list[detected_label] = 1
        if time_module.time() - self.start_time > 5:
            obj_name = ""
            for key in self.obj_list.keys():
                print(key,self.obj_list[key])
                if self.obj_list[key] > 5:
                    obj_name = f"{obj_name} {key}"
            self.obj_list = {}
            return obj_name
        else:
            return None

    def do_pos(self, object, results, obj_feature):
        """
        返回object对应feature的物品名称和位置
        每帧筛选距离目标最近的符合特征物体（基于xyz距离），统计标签出现次数
        """
        # 初始化
        if not hasattr(self, 'target_obj'):
            self.target_obj = None
            self.label_stats = {}  # 结构: {label: {'count': int, 'last_pos': pos}}

        # 寻找目标物体
        self.target_obj = None
        for res in results:
            if res['pos'][2] < 2 and is_target_kind(object, res['label']):
                self.target_obj = {'label': res['label'], 'pos': res['pos']}
                break

        if not self.target_obj:
            return None, None

        # 筛选逻辑
        closest_obj = None
        min_distance = float('inf')
        target_pos = self.target_obj['pos']

        for res in results:
            label, pos = res['label'], res['pos']
            # 跳过无效物体和目标物体本身
            if label == self.target_obj['label'] or pos[2] >= 2:
                continue

            # 特征条件判断
            is_feature_match = False
            if obj_feature == "left" and pos[0] < target_pos[0]:
                is_feature_match = True
            elif obj_feature == "right" and pos[0] > target_pos[0]:
                is_feature_match = True
            elif obj_feature in ["top", "above"] and pos[1] < target_pos[1]:
                is_feature_match = True
            elif obj_feature == "behind" and pos[2] > target_pos[2]:
                is_feature_match = True
            elif obj_feature == "under" and pos[1] > target_pos[1]:
                is_feature_match = True

            # 计算距离并更新最近物体
            if is_feature_match:
                distance = ((pos[0] - target_pos[0]) ** 2 +
                            (pos[1] - target_pos[1]) ** 2 +
                            (pos[2] - target_pos[2]) ** 2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    closest_obj = (label, pos)

        # 更新统计（每帧只记录最近物体）
        if closest_obj:
            label, pos = closest_obj
            if label not in self.label_stats:
                self.label_stats[label] = {'count': 0, 'last_pos': None}
            self.label_stats[label]['count'] += 1
            self.label_stats[label]['last_pos'] = pos  # 始终记录最近的位置

        # 超时处理（10秒）
        if time_module.time() - self.start_time > 10:
            if not self.label_stats:
                print("⚠️ 未检测到有效物体")
                result = (None, None)
            else:
                # 打印统计信息（按频率排序）
                print(f"\n【{obj_feature}物体统计】")
                for label, stats in sorted(self.label_stats.items(),
                                           key=lambda x: x[1]['count'],
                                           reverse=True):
                    print(f"  {label}: 出现{stats['count']}次")

                # 选择最高频的标签
                most_common = max(self.label_stats.items(),
                                  key=lambda x: x[1]['count'])
                result = (most_common[0], most_common[1]['last_pos'])
                print(f"✅ 最终选择: {result[0]} (位置: {result[1]})")

            # 重置状态
            self.target_obj = None
            self.label_stats = {}
            self.start_time = time_module.time()
            return result
        else:
            return None, None

    def do_pos_most(self, object, results, obj_feature):
        """
        返回最左/最右物品
        每一帧先筛选最左/最右物体，再统计标签出现次数
        """
        # 初始化统计字典
        if not hasattr(self, 'label_stats'):
            self.label_stats = {}  # 结构: {label: {'count': int, 'last_pos': list}}

        # 当前帧的物体列表
        current_objs = [
            (res['label'], res['pos'])
            for res in results
            if res['pos'][2] < 2
        ]

        # 如果当前帧有物体，筛选最左/最右
        if current_objs:
            if obj_feature == 'leftmost':
                selected_label, selected_pos = min(current_objs, key=lambda x: x[1][0])  # x坐标最小
            else:  # 'rightmost'
                selected_label, selected_pos = max(current_objs, key=lambda x: x[1][0])  # x坐标最大

            # 更新统计
            if selected_label not in self.label_stats:
                self.label_stats[selected_label] = {'count': 0, 'last_pos': None}
            self.label_stats[selected_label]['count'] += 1
            self.label_stats[selected_label]['last_pos'] = selected_pos  # 更新为最新位置

        # 检查超时（10秒）
        if time_module.time() - self.start_time > 10:
            if not self.label_stats:
                print("没有检测到任何物体")
                result = (None, None)
            else:
                # 打印统计信息
                print(f"\n{obj_feature}统计（按帧筛选）：")
                for label, stats in sorted(self.label_stats.items(),
                                           key=lambda x: x[1]['count'], reverse=True):
                    print(f"物体: {label}, 出现次数: {stats['count']}")

                # 选择出现次数最多的标签
                most_common_label = max(self.label_stats.items(),
                                        key=lambda x: x[1]['count'])[0]
                result_pos = self.label_stats[most_common_label]['last_pos']

                result = (most_common_label, result_pos)
                print(
                    f"最终选择: {result[0]} (出现{self.label_stats[most_common_label]['count']}次), 位置: {result[1]}")

            # 重置统计
            self.label_stats = {}
            self.start_time = time_module.time()
            return result
        else:
            return None, None

if __name__ == "__main__":
    #################### 启动ros ########################
    rospy.init_node('FIND_OBJECT_CAMERA_NODE')
    # ROS发布器
    image_pub = rospy.Publisher("/gpsr_img", sensor_msgs.msg.Image, queue_size=10)
    pub = rospy.Publisher("/find_object_result",object_position,queue_size=10)
    sub = rospy.Subscriber("/findobject_camera_state", String, CameraStateCallback)
    # 发布坐标信息
    p = object_position()
    rate = rospy.Rate(10) # 10hz

    fps = 0.0
    yolo = ObjDetect()
    obj_analyse = ObjAnalyse()
    bridge = CvBridge()
    try:
        while not rospy.is_shutdown():
            #这个判断条件应该要改，根据相机开启流返回的数据判断是否进行处理，不然有一定可能在pipeline.wait_for_frames()报错
            if camera.camera_state == True:
                pTime=time_module.time()
                ##################### 相机启动 ########################
                color_image, depth_image, depth_intrin, depth_frame = camera.get_frame()
                obj_feature =  get_info("obj_feature")
                obj_target = get_info("obj_target")
                if depth_frame  is None or color_image is None:
                    continue

                #img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                img = color_image
                if camera_state == True:
                    # 进行检测
                    results = yolo.get_obj(img,depth_frame,depth_intrin)
                    if obj_feature is not None and obj_feature == "count":
                        obj_analyse.update_time()
                        obj_count  = obj_analyse.do_count(obj_target,results)
                        if obj_count is not None:
                            rospy.loginfo(f"the object counted is {obj_count}")
                            rospy.set_param("obj_count",obj_count)
                            obj_analyse.is_analyze = False
                            rospy.set_param("obj_feature", "none")
                            rospy.set_param("obj_target", "none")
                        yolo.draw_box(img,results)
                    elif obj_feature is not None and obj_feature in ["biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"]:
                        obj_analyse.update_time()
                        obj_name = obj_analyse.do_analyse(obj_target,results)
                        if obj_name is not None:
                            rospy.loginfo(f"the object detected is {obj_name}")
                            rospy.set_param("obj_name",obj_name)
                            obj_analyse.is_analyze = False
                            rospy.set_param("obj_feature", "none")
                            rospy.set_param("obj_target", "none")
                        yolo.draw_box(img, results)
                    elif obj_feature is not None and obj_feature in  ["left", "right", 'top', "above", 'behind', 'under']:
                        obj_analyse.update_time()
                        obj_name,pos = obj_analyse.do_pos(obj_target,results,obj_feature)
                        if obj_name is not None:
                            rospy.loginfo(f"the object detected is {obj_name}")
                            rospy.set_param("obj_name",obj_name)
                            obj_analyse.is_analyze = False
                            rospy.set_param("obj_feature", "none")
                            rospy.set_param("obj_target", "none")
                            p.label = obj_name
                            p.x = pos[0]
                            p.y = pos[1]
                            p.z = pos[2]
                            pub.publish(p)
                        yolo.draw_box(img, results)
                    elif obj_feature is not None and obj_feature in  ['leftmost', 'rightmost']:
                        obj_analyse.update_time()
                        obj_name,pos = obj_analyse.do_pos_most(obj_target,results,obj_feature)
                        if obj_name is not None:
                            rospy.loginfo(f"the object detected is {obj_name}")
                            rospy.set_param("obj_name",obj_name)
                            obj_analyse.is_analyze = False
                            p.label = obj_name
                            p.x = pos[0]
                            p.y = pos[1]
                            p.z = pos[2]
                            pub.publish(p)
                            rospy.set_param("obj_feature", "none")
                            rospy.set_param("obj_target", "none")
                        yolo.draw_box(img, results)
                    else:
                        #逐个返回所有物品
                        for res in results:
                            detected_label = res['label']
                            x1, y1, x2, y2 = res['box']
                            pos = res['pos']
                            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(img, f"Label:{detected_label} X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}",
                                        ((x1 + x2) // 2 + 10, (y1 + y2) // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                        (0, 255, 0), 2)
                            p.label = detected_label
                            p.x = pos[0]
                            p.y = pos[1]
                            p.z = pos[2]

                            # 坐标点发布，不发布识别结果为0的点
                            if p.x ** 2 + p.y ** 2 + p.z ** 2 > 0.01:
                                pub.publish(p)
                                rospy.loginfo('%s:(%f,%f,%f)' % (p.label, p.x, p.y, p.z))
                            else:
                                rospy.logwarn('%s:(%f,%f,%f)' % (p.label, p.x, p.y, p.z))

                # 显示帧率
                cTime=time_module.time()
                fps = 1 / (cTime - pTime)
                pTime = cTime
                cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
                # cv转img
                ros_image = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                # 发布图片消息
                image_pub.publish(ros_image)  

    except rospy.ROSInterruptException:
        pass