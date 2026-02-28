#! /home/syw/anaconda3/envs/GPSR/bin/python
# -*- coding: utf-8 -*-
import cv2
import time
import pyrealsense2 as rs
import numpy as np
# from PIL import Image
import face_recognition
from deepface import DeepFace
import datetime

import rospy
from std_msgs.msg import String
from seu_robot_msgs.msg import object_position

import sensor_msgs.msg
from cv_bridge import CvBridge

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
# 一个相机识别并发布坐标
camera_state = False
analyze_state = False
analyze_count = 0

# 初始化Mediapipe人体骨架跟踪
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# 初始化MediaPipe手部骨架检测
mp_hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# 计算像素点深度与三维坐标
def get_3d_camera_coordinate(x, y, aligned_depth_frame, depth_intrin):
    pix_x=int(x)
    pix_y=int(y)
    dis = aligned_depth_frame.get_distance(pix_x,pix_y)        # 获取该像素点对应的深度
    #print ('depth: ',dis)       # 深度单位是m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [pix_x,pix_y], dis)
    # print ('camera_coordinate: ',camera_coordinate)
    return dis, camera_coordinate


# 接收信号，决定是否开始检测
def CameraStateCallback(msg):
    global camera_state
    global camera_state_last
    global analyze_state
    global analyze_count
    global camera_feature

    camera_state_last = camera_state
    camera_feature = msg.data
    #判断检测哪个特征：
    #识别人：face_recognition,识别动作：mediapipe
    if msg.data == "camera_open" or msg.data == "camera_feature":
        camera_state = True
        analyze_state = False
    #识别性别：采用deepface
    elif msg.data == "camera_analyze":
        camera_state = True
        analyze_state = True
    #关闭
    elif msg.data == "camera_close":
        camera_state = False
        analyze_state = False

    #开启视频流
    if camera_state_last == False:
        if camera_state == True:
            profile = pipeline.start(config)
            p.label = "person"
            rospy.logwarn('Start the analysis of finding person') 
            analyze_count = 0
            
    #关闭视频流
    elif camera_state_last == True:
        if camera_state == False:
            cv2.destroyAllWindows() 
            pipeline.stop()
            rospy.logwarn('Close the analysis of finding person')


if __name__ == "__main__":
    #################### 启动ros ########################
    rospy.init_node('FIND_PERSON_CAMERA')
    # ROS发布器
    image_pub = rospy.Publisher("/gpsr_img", sensor_msgs.msg.Image, queue_size=10)
    pub = rospy.Publisher("/find_person_result",object_position,queue_size=10)
    sub = rospy.Subscriber("/findperson_camera_state", String, CameraStateCallback)
    # 发布坐标信息
    p = object_position()
    # p.label = "person"
    rate = rospy.Rate(10) # 10hz

    #################### 配置相机 ########################
    fps = 0.0
    # 相机RGB和深度图对齐
    align = rs.align(rs.stream.color)
    # 配置视频流
    config = rs.config()
    print(len(rs.context().query_devices()))
    if 1 < len(rs.context().query_devices()):
        config.enable_device(rs.context().query_devices()[1].get_info(rs.camera_info.serial_number))
    else:
        config.enable_device(rs.context().query_devices()[0].get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # 创建一个上下文对象。该对象拥有所有连接的Realsense设备的句柄
    pipeline = rs.pipeline()

    #参数初始化
    image_width = 640
    image_height = 480
    Frame_Num = 0
    left_h_distance_pre = 0
    right_h_distance_pre = 0
    a, b, c, d, e, f, g, h = (0, 0, 0, 0, 0, 0, 0, 0)
    raise_arm = "None"
    wave_hand = "None"

    try:
        while not rospy.is_shutdown():
            #这个判断条件应该要改，根据相机开启流返回的数据判断是否进行处理，不然有一定可能在pipeline.wait_for_frames()报错
            if camera_state == True:
                pTime=time.time()
                ##################### 相机启动 ########################
                frames = pipeline.wait_for_frames()
                # 将RGBD对齐
                aligned_frames = align.process(frames)

                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not depth_frame or not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics     # 获取深度参数（像素坐标系转相机坐标系会用到）
                color_intrin = color_frame.profile.as_video_stream_profile().intrinsics     # 获取相机内参
                
                img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                # 转变成Image
                #img = Image.fromarray(np.uint8(img))
                if analyze_state == False:
                    if camera_feature == "camera_open":
                        # 进行检测：获取最大脸
                        face_locations = face_recognition.face_locations(img)
                        if len(face_locations):
                            max_face_area = 0
                            max_face = []
                            for face_location in face_locations:
                                top, right, bottom, left = face_location
                                if not max_face:
                                    if max_face_area <  (right - left) * (bottom - top):
                                        max_face = face_location
                                        max_face_area = (right - left) * (bottom - top)
                                else:
                                    max_face = face_location
                                    max_face_area = (right - left) * (bottom - top)
                                    
                            cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
                            #timestamp = time.asctime()
                            #cv2.imwrite(str(timestamp) + '.jpg', img)
                            # 输出基于摄像头的坐标        
                            dist, result=get_3d_camera_coordinate((right + left)/2, (bottom + top)/2, depth_frame, depth_intrin)
                            p.x = result[0]
                            p.y = result[1]
                            p.z = result[2]

                            # 坐标点发布，不发布识别结果为0的点
                            if (p.x**2+p.y**2+p.z**2>0.1) :
                                pub.publish(p)
                                rospy.loginfo('%s:(%f,%f,%f)' %(p.label,p.x,p.y,p.z))
                            else:
                                rospy.logwarn('%s:(%f,%f,%f)' %(p.label,p.x,p.y,p.z))
                    elif camera_feature == "camera_feature":
                        # 进行手势识别
                        hands_results = mp_hands.process(img)
                        
                        ############################### Pointing ###############################
                        # 检测到的手势
                        point_pos = 'None'
                        if hands_results.multi_hand_landmarks:
                            for hand_landmarks in hands_results.multi_hand_landmarks:
                                # 获取第食指关节的坐标
                                index_finger_mcp = hand_landmarks.landmark[5]
                                index_finger_tip = hand_landmarks.landmark[8]
                                middle_finger_tip = hand_landmarks.landmark[12]
                                ring_finger_tip = hand_landmarks.landmark[16]
                                pinky_tip = hand_landmarks.landmark[20]

                                # 判断手指是否指向正左（右）方
                                # print(abs(index_finger_mcp.y - index_finger_tip.y))
                                # print(index_finger_mcp.x - index_finger_tip.x)
                                if index_finger_tip.x > 0 and index_finger_tip.x < 1:
                                    if abs(index_finger_mcp.y - index_finger_tip.y) < 0.05:
                                        if index_finger_mcp.x > index_finger_tip.x and \
                                            index_finger_mcp.x < middle_finger_tip.x and \
                                            index_finger_mcp.x < ring_finger_tip.x and \
                                            index_finger_mcp.x < pinky_tip.x:
                                                dist, result=get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                                                p = object_position()
                                                p.label = 'point_right'
                                                p.x = result[0]
                                                p.y = result[1]
                                                p.z = result[2]
                                                pub.publish(p)
                                                point_pos = 'Right'
                                        elif index_finger_mcp.x < index_finger_tip.x and \
                                            index_finger_mcp.x > middle_finger_tip.x and \
                                            index_finger_mcp.x > ring_finger_tip.x and \
                                            index_finger_mcp.x > pinky_tip.x:
                                                dist, result=get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                                                p = object_position()
                                                p.label = 'point_left'
                                                p.x = result[0]
                                                p.y = result[1]
                                                p.z = result[2]
                                                pub.publish(p)
                                                point_pos = 'Left'
                                # 绘制手部关键点
                                # mp_drawing.draw_landmarks(img, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
                        cv2.putText(img, f'Finger Pointing: {point_pos}', (10, 60),
                                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                        #timestamp = time.asctime()

                        #cv2.imwrite(str(timestamp) + '.jpg', img)

                        # 进行骨架识别
                        pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
                        pose_results = pose.process(img)
                        
                        # 绘制骨架
                        # mp_drawing.draw_landmarks(img, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                        
                        # 获取肩部关键点
                        if pose_results.pose_landmarks:
                            # 获取关键点坐标
                            landmarks = pose_results.pose_landmarks.landmark
                            
                            ############################### Sitting ###############################
                            # 判断是否符合坐姿条件
                            # 这里可以根据你的需求和具体场景进行定制化的判断条件
                            # print(abs(landmarks[mp_pose.PoseLandmark.LEFT_KNEE].y - landmarks[mp_pose.PoseLandmark.LEFT_HIP].y))
                            if abs(landmarks[mp_pose.PoseLandmark.LEFT_KNEE].y - landmarks[mp_pose.PoseLandmark.LEFT_HIP].y) < 0.3 and \
                                abs(landmarks[mp_pose.PoseLandmark.RIGHT_KNEE].y - landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y) < 0.3 :
                                is_sitting = True
                            else:
                                is_sitting = False

                            # 在帧上绘制检测到的关键点和坐姿结果
                            cv2.putText(img, "Sitting: " + str(is_sitting), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                            
                            #timestamp = time.asctime()
                            #cv2.imwrite(str(timestamp) + '.jpg', img)
                            
                            ############################### Raise and Wave ###############################
                            
                            # 假设你已经获取到了手腕和肩的关键点
                            left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
                            right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
                            
                            left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]
                            right_wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
                            
                            # rospy.loginfo(f'left:{left_wrist},right:{right_wrist}')

                            # 超限处理
                            left_wrist.x, left_wrist.y = (left_wrist.x, left_wrist.y) if (left_wrist.x >= 0 and left_wrist.x < 1 and left_wrist.y >= 0 and left_wrist.y < 1) else (0.99, 0.99)
                            right_wrist.x, right_wrist.y = (right_wrist.x, right_wrist.y) if (right_wrist.x >= 0 and right_wrist.x < 1 and right_wrist.y >= 0 and right_wrist.y < 1) else (0.99, 0.99)
                            
                            cv2.circle(img, (int(left_shoulder.x * image_width), int(left_shoulder.y * image_height)), 5, (0, 255, 255), -1)
                            cv2.circle(img, (int(right_shoulder.x * image_width), int(right_shoulder.y * image_height)), 5, (255, 255, 0), -1)
                            cv2.circle(img, (int(left_wrist.x * image_width), int(left_wrist.y * image_height)), 5, (0, 255, 255), -1)
                            cv2.circle(img, (int(right_wrist.x * image_width), int(right_wrist.y * image_height)), 5, (255, 255, 0), -1)
                            # 计算两肩的中点坐标
                            mid_point_x = (left_shoulder.x + right_shoulder.x) / 2
                            mid_point_y = (left_shoulder.y + right_shoulder.y) / 2
                            # 将归一化坐标转换为像素坐标
                            image_width = 640
                            image_height = 480
                            pixel_x = int(mid_point_x * image_width)
                            pixel_y = int(mid_point_y * image_height)
                            center = (pixel_x, pixel_y)
                            # rospy.loginfo(f'left:{left_wrist},right:{right_wrist}')

                            # 获取左右手相对于左右肩的垂直位置
                            left_v_distance = left_wrist.y - left_shoulder.y
                            right_v_distance = right_wrist.y - right_shoulder.y
                            
                            # 获取左右手相对于左右肩的水平位置
                            left_h_distance = left_wrist.x - left_shoulder.x
                            right_h_distance = right_wrist.x - right_shoulder.x
                            
                            # 计算左右手的速度
                            if Frame_Num % 2 ==0:
                                left_h_distance_pre = left_h_distance
                                right_h_distance_pre = right_h_distance
                                
                            Frame_Num += 1
                            
                            left_h_vel = left_h_distance_pre - left_h_distance
                            right_h_vel = right_h_distance_pre - right_h_distance

                            if Frame_Num % 2 ==0:
                                # 判断左右手是否举起
                                if left_v_distance < 0 and right_v_distance < 0:
                                    a += 1
                                    # rospy.loginfo("左右手同时举起！")
                                elif left_v_distance < 0:
                                    b += 1
                                    # rospy.loginfo("左手举起。")
                                    dist, result=get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                                    p = object_position()
                                    p.label = 'raise_left'
                                    p.x = result[0]
                                    p.y = result[1]
                                    p.z = result[2]
                                    pub.publish(p)
                                elif right_v_distance < 0:
                                    c += 1
                                    # rospy.loginfo("右手举起。")
                                    dist, result=get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                                    p = object_position()
                                    p.label = 'raise_right'
                                    p.x = result[0]
                                    p.y = result[1]
                                    p.z = result[2]
                                    pub.publish(p)
                                else:
                                    d += 1
                                    # rospy.loginfo("左右手均未举起。")
                                # 判断是否挥手
                                threshold = 0.01
                                if abs(left_h_vel) > threshold and abs(right_h_vel) > threshold:
                                    e += 1
                                    # rospy.loginfo("左右手同时挥手！")
                                    dist, result=get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                                    p = object_position()
                                    p.label = 'wave'
                                    p.x = result[0]
                                    p.y = result[1]
                                    p.z = result[2]
                                    pub.publish(p)
                                elif abs(left_h_vel) > threshold:
                                    f += 1
                                    # rospy.loginfo("左手挥手。")
                                    dist, result=get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                                    p = object_position()
                                    p.label = 'wave'
                                    p.x = result[0]
                                    p.y = result[1]
                                    p.z = result[2]
                                    pub.publish(p)
                                elif abs(right_h_vel) > threshold:
                                    g += 1
                                    # rospy.loginfo("右手挥手。")
                                    dist, result=get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                                    p = object_position()
                                    p.label = 'wave'
                                    p.x = result[0]
                                    p.y = result[1]
                                    p.z = result[2]
                                    pub.publish(p)
                                else:
                                    h += 1
                                    # rospy.loginfo("左右手均未挥手。")
                                    
                                
                            # 更新
                            left_h_distance_pre = left_h_distance
                            right_h_distance_pre = right_h_distance
                            
                            if Frame_Num % 5 == 0:
                                variables_1 = {'a': a, 'b': b, 'c': c, 'd': d}
                                max_variable_1 = max(variables_1, key=variables_1.get)
                                variables_2 = {'e': e, 'f': f, 'g': g, 'h': h}
                                max_variable_2 = max(variables_2, key=variables_2.get)
                                if max_variable_1 == 'a':
                                    # rospy.logerr("左右手同时举起！")
                                    raise_arm = "Left and Right"
                                elif max_variable_1 == 'b':
                                    # rospy.logwarn("左手举起。")
                                    raise_arm = "Left"
                                elif max_variable_1 == 'c':
                                    # rospy.logwarn("右手举起。")
                                    raise_arm = "Right"
                                elif max_variable_1 == 'd':
                                    # rospy.loginfo("左右手均未举起。")
                                    raise_arm = "None"
                                if max_variable_2 == 'e':
                                    # rospy.logerr("左右手同时挥手！")
                                    wave_hand = "Left and Right"
                                elif max_variable_2 == 'f':
                                    # rospy.logwarn("左手挥手。")
                                    wave_hand = "Left"
                                elif max_variable_2 == 'g':
                                    # rospy.logwarn("右手挥手。")
                                    wave_hand = "Right"
                                elif max_variable_2 == 'h':
                                    # rospy.loginfo("左右手均未挥手。")
                                    wave_hand = "None"
                                # rospy.loginfo('------------------------')
                                a, b, c, d, e, f, g, h = (0, 0, 0, 0, 0, 0, 0, 0)
                                
                            cv2.putText(img, "Raised Arm: " + str(raise_arm), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)
                            cv2.putText(img, "Waved Hand: " + str(wave_hand), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                            timestamp = time.asctime()

                            cv2.imwrite(str(timestamp) + '.jpg', img)
                        # else:
                            # rospy.logwarn('未识别到人')
                #性别判断
                elif analyze_state == True:
                    analyze_count = analyze_count + 1
                    if analyze_count == 10 :
                        analysis = DeepFace.analyze(img_path=img, actions=["gender"])
                        #图片中识别多个人只要有一个性别满足条件就返回成功
                        for info in analysis:
                            print(info)
                            if info['dominant_gender'] == 'Woman':
                                p.label = "woman"
                                rospy.loginfo('%s:(%f,%f,%f)' %(p.label,p.x,p.y,p.z))
                                pub.publish(p)
                                break
                            else:
                                p.label = "man"
                                rospy.loginfo('%s:(%f,%f,%f)' %(p.label,p.x,p.y,p.z))
                                pub.publish(p)
                                break

                img = np.array(img)
                # RGBtoBGR满足opencv显示格式
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


                # 显示帧率
                cTime=time.time()
                fps = 1 / (cTime - pTime)
                pTime = cTime
                cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
                
                # 创建CvBridge对象
                bridge = CvBridge()
                # cv转img
                ros_image = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                # 发布图片消息
                image_pub.publish(ros_image)  

                # cv2.imshow("Image", img)
                # if cv2.waitKey(1) == ord("q"):
                #     break

    except rospy.ROSInterruptException:
        pipeline.stop()
        pass