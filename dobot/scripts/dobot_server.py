#! /home/syw/anaconda3/envs/GPSR/bin/python
# -*- coding: utf-8 -*-

# written by zzn
# 机械臂服务名为/dobot_control,具体格式可见软件包dobot下的srv文件夹中dobot_control.srv
# 订阅消息/detect_result2,内容为俯视摄像头坐标系下的物体坐标，使用自定义消息类型object_position,存放在软件包robot_msgs的msg文件夹中
import DobotDllType as dType
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
import time
import math
# import tf
# import geometry_msgs.msg
import rospy
from dobot.srv import dobot_control
from robot_msgs.msg import object_position
import json
import numpy as np
import pyttsx3
global flag

catch_width = 0.65
wait_width = 0.52
open_width = 0.3
carry_wait_width = 0.55
carry_width = 0.72
catch_height = 5
up_height = 60

flag = 0
# 语音播报
def speech_to_text(data):
    speech=pyttsx3.init()
    voices=speech.getProperty('voices')
    speech.setProperty('voice',voices[10].id)
    speech.setProperty('rate',160)
    speech.say(data)
    speech.runAndWait()

item_name = 'item'
item_name_tmp = 'item'
item_count = -1

# 类定义
class DobotControl:
    def __init__(self):
        self.sub_multi = rospy.Subscriber("/detect_result2",object_position,self.arm_callpack,queue_size=10)
        # self.sub_obj = rospy.Subscriber("object_pos",object_position,self.arm_callpack,queue_size=10)
        self.sub_place = rospy.Subscriber("/sg_place_position",object_position,self.arm_callpack,queue_size=10)
        self.p =object_position()

        # 机械臂初始化
        CON_STR = {
        dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
        dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
        dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

        self.api = dType.load()
        self.state = dType.ConnectDobot(self.api, "", 115200)[0]
        rospy.loginfo("Connect status:{}".format(CON_STR[self.state]))
        if (self.state != dType.DobotConnect.DobotConnect_NoError):
            return
        rospy.loginfo('robot state is:{}'.format(dType.GetPose(self.api)))

    # 订阅物体坐标的回调函数
    def arm_callpack(self,msg):
        global item_name, item_name_tmp, item_count
        self.p.label=msg.label
        self.p.x=msg.x
        self.p.y=msg.y
        self.p.z=msg.z
        item_name_tmp = msg.label
        if item_count == -1:
            item_name = item_name_tmp
            item_count += 1
        if item_name != msg:
            item_count += 1
        if item_count > 5:
            item_name = item_name_tmp
            item_count = -1



    # 抓取位置坐标转换
    def dobot_calculate(self):
        # 手眼标定矩阵top camera 1
        image_to_arm=  [[ 5.54378121e+01, -3.03141594e+02,  7.04487977e+02, -2.49733362e+02],
                        [-4.20454918e+02,  8.77792639e+01,  2.05512386e+02, -8.60513871e+01],
                        [ 3.75706051e+00, -8.67924102e+02, -4.69569766e+02,  3.55620486e+02],
                        [ 6.66133815e-16,  7.99360578e-15,  0.00000000e+00,  1.00000000e+00]]
        
        pt=[self.p.x, self.p.y, self.p.z,1.0]
        dobotpt=np.dot(image_to_arm, np.array(pt))[0:3]
        return dobotpt
    
    # 清除机械臂报警
    def dobot_clear(self):
        dType.ClearAllAlarmsState(self.api)
        
    # 机械臂复位
    def dobot_reset(self):
        rospy.loginfo("reset")

        #清空队列
        dType.SetQueuedCmdClear(self.api)

        #设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        
        #开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        #设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]

        # 张开手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True
        
    # 机械臂转到一侧去
    def dobot_turn(self):
        rospy.loginfo("turn")
        #清空队列
        dType.SetQueuedCmdClear(self.api)

        #设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        
        #开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        #设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)

        #机械臂转到一侧，防止遮挡物体，影响识别
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 0, -120, 30, 0, isQueued = 1)[0]

        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True

    # 机械臂断开连接
    def dobot_disconnect(self):
        disconnect = dType.DisconnectDobot(self.api)
        return disconnect

    # 机械臂伸到目标物体上方并语音播报申请辅助
    def dobot_reach(self):
        global item_name
        rospy.loginfo("reach")
        dobotpt=self.dobot_calculate()
        rospy.loginfo(dobotpt)
        # 判断目标点是否超出范围
        ptlen = math.sqrt(dobotpt[0]**2+dobotpt[1]**2)
        # 230, 180, 292,
        if ptlen>250:
            dobotpt[0]=int(dobotpt[0]*250/ptlen)
            dobotpt[1]=int(dobotpt[1]*250/ptlen)

        # 清空队列
        dType.SetQueuedCmdClear(self.api)

        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued=1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued=1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        dType.SetEndEffectorParams(self.api, 0, 0, 0, isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 100, 0, 0, 0, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 盲伸位置
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 177, 2, 63, 0, isQueued=1)[0]
        rospy.loginfo('robot state is:{}'.format(dType.GetPose(self.api)))
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        speech_to_text('please help me catch the')
        speech_to_text(item_name)
        lastIndex = dType.SetWAITCmd(self.api, 2, isQueued=1)[0]

        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, 0.7, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        dType.SetEndEffectorParams(self.api, 0, 0, 0, isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, 30, 0, isQueued=1)[0]

        # 如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        # 停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True

    # 俯视摄像头识别不到时盲伸
    def dobot_move(self):
        global item_name, flag
        if item_name == 'water glass':
            dC = 0.7
        else:
            dC = 0.75
        rospy.loginfo("move")
        rospy.loginfo("盲抓")
        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        
        # 张开手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 盲伸位置
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 231.823, 2.999, 8.718, 0, isQueued = 1)[0]
        rospy.loginfo('robot state is:{}'.format(dType.GetPose(self.api)))
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        speech_to_text('please help me catch the')
        speech_to_text(item_name)
        flag = 0
        lastIndex = dType.SetWAITCmd(self.api, 2, isQueued=1)[0]

        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, dC,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, 30, 0, isQueued = 1)[0]
        
        # 如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        # 停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True

    # 机械臂收回
    def dobot_back(self):
        rospy.loginfo("back")

        #清空队列
        dType.SetQueuedCmdClear(self.api)
        
        #设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        #开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        #设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, 0.7,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]

        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True

    # storing groceries机械臂抓取1
    def dobot_groceries_catch1(self):
        rospy.loginfo(">>>storing groceries机械臂抓取1")
        dobotpt = self.dobot_calculate()
        rospy.loginfo(dobotpt)
        # speech_to_text('to catch the item')
        # 判断是否超限
        ptlen = math.sqrt(dobotpt[0]**2+dobotpt[1]**2)
        if ptlen>250:
            speech_to_text('out of range')
            dobotpt[0] = int(dobotpt[0]*250/ptlen)
            dobotpt[1] = int(dobotpt[1]*250/ptlen)
        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        # 张开手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        # 伸向物体
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, (dobotpt[0]+10), dobotpt[1]-2, catch_height, 0, isQueued=1)
        # lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 177, 2, 63, 0, isQueued=1)
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 闭合手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, catch_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        print("going to up")
        # 稍微上升手爪
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, (dobotpt[0]+10), dobotpt[1]-2, catch_height+10, 0, isQueued=1)
        lastIndex = dType.SetWAITCmd(self.api, 1.5, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1.5, isQueued=1)[0]
        # # 回位
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        # lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True
        
    # storing groceries机械臂抓取2
    def dobot_groceries_catch2(self):
        rospy.loginfo(">>>storing groceries机械臂抓取2")

        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)

        # 回位
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, up_height, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True
    
    # storing groceries机械臂放置
    def dobot_place(self):
        rospy.loginfo(">>>storing groceries机械臂放置")
        dobotpt = self.dobot_calculate()
        rospy.loginfo(dobotpt)
        # speech_to_text('to place the item')
        # 判断是否超限
        ptlen = math.sqrt(dobotpt[0]**2+dobotpt[1]**2)
        if ptlen>250:
            speech_to_text('out of range')
            dobotpt[0] = int(dobotpt[0]*250/ptlen)
            dobotpt[1] = int(dobotpt[1]*250/ptlen)
        # 清空队列
        dType.SetQueuedCmdClear(self.api)

        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)

        # 伸向位置上侧
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 160, 0, up_height, 0, isQueued=1)
        # lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 177, 2, 63, 0, isQueued=1)
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 松开手爪稳定
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, wait_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        time.sleep(3)

        # 松开手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 回位
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued=1)[0]

        # 如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        # 停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True

    # # storing groceries机械臂放置
    # def dobot_place1(self):
    #     rospy.loginfo(">>>storing groceries机械臂放置1")
    #     dobotpt = self.dobot_calculate()
    #     rospy.loginfo(dobotpt)
    #     # speech_to_text('to place the item')
    #     # 判断是否超限
    #     ptlen = math.sqrt(dobotpt[0]**2+dobotpt[1]**2)
    #     if ptlen>250:
    #         speech_to_text('out of range')
    #         dobotpt[0] = int(dobotpt[0]*250/ptlen)
    #         dobotpt[1] = int(dobotpt[1]*250/ptlen)
    #     # 清空队列
    #     dType.SetQueuedCmdClear(self.api)

    #     # 设置运动参数
    #     dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
    #     dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    #     dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

    #     # 开始执行指令队列
    #     dType.SetQueuedCmdStartExec(self.api)

    #     # 设置ptpcmd内容并将命令发送给dobot
    #     result = dType.SetCmdTimeout(self.api, 100)

    #     # 伸向位置上侧
    #     lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 160, 0, up_height, 0, isQueued=1)
    #     # lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 177, 2, 63, 0, isQueued=1)
    #     lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

    #     # 如果还未完成指令队列则等待
    #     while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
    #         dType.dSleep(10)

    #     print('timeout:', result)

    #     # 停止执行指令
    #     dType.SetQueuedCmdStopExec(self.api)
    #     dType.SetQueuedCmdClear(self.api)
    #     if result != None:
    #         return False
    #     else:
    #         return True
        
    #     # storing groceries机械臂放置
    # def dobot_place2(self):
    #     rospy.loginfo(">>>storing groceries机械臂放置2")
    #     # 清空队列
    #     dType.SetQueuedCmdClear(self.api)

    #     # 设置运动参数
    #     dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
    #     dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    #     dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

    #     # 开始执行指令队列
    #     dType.SetQueuedCmdStartExec(self.api)

    #     # 设置ptpcmd内容并将命令发送给dobot
    #     result = dType.SetCmdTimeout(self.api, 100)

    #     # 松开手爪稳定
    #     lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
    #     lastIndex = dType.SetIOPWM(self.api, 4, 5, wait_width,isQueued=1)[0]
    #     lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

    #     time.sleep(3)

    #     # 松开手爪
    #     lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
    #     lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
    #     lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

    #     # 回位
    #     lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued=1)[0]

    #     # 如果还未完成指令队列则等待
    #     while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
    #         dType.dSleep(10)

    #     print('timeout:', result)

    #     # 停止执行指令
    #     dType.SetQueuedCmdStopExec(self.api)
    #     dType.SetQueuedCmdClear(self.api)
    #     if result != None:
    #         return False
    #     else:
    #         return True

    # carry my luggage机械臂抓取1
    def dobot_carry_catch1(self):
        rospy.loginfo(">>>carry my luggage机械臂抓取1")
        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        # 张开手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, carry_wait_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        # 伸向物体
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 177, 2, 63, 0, isQueued=1)
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True
        
    # carry my luggage机械臂抓取2
    def dobot_carry_catch2(self):
        rospy.loginfo(">>>carry my luggage机械臂抓取2")

        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)
        
        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 闭合手爪
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, carry_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 回位
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True
    
    # carry my luggage机械臂抓放置1
    def dobot_carry_give1(self):
        rospy.loginfo(">>>carry my luggage机械臂抓放置1")
        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 伸向物体
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 177, 2, 63, 0, isQueued=1)
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 张开手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, carry_wait_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True
        
    # carry my luggage机械臂抓放置2
    def dobot_carry_give2(self):
        rospy.loginfo(">>>carry my luggage机械臂抓放置2")

        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 完全张开手爪
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 回位
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True
        
    # gpsr机械臂抓放置1
    def dobot_gpsr_give1(self):
        rospy.loginfo(">>>gpsr机械臂抓放置1")
        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 伸向物体
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 177, 2, 63, 0, isQueued=1)
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True
        
    # gpsr机械臂抓放置2
    def dobot_gpsr_give2(self):
        rospy.loginfo(">>>gpsr机械臂抓放置2")

        # 清空队列
        dType.SetQueuedCmdClear(self.api)
        
        # 设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)

        # 开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        # 设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 完全张开手爪
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]

        # 回位
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # speech_to_text('give you')
        if result != None:
            return False
        else:
            return True

    # 机械臂扔垃圾
    def dobot_drop(self):
        rospy.loginfo("drop")

        #清空队列
        dType.SetQueuedCmdClear(self.api)
        
        #设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)

        #开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        #设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        # 往前伸
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 231.823, 2.999, 8.718, 0, isQueued = 1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        # 松开手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, open_width,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        # 闭合手爪
        lastIndex = dType.SetIOMultiplexing(self.api, 4, 2, isQueued=1)
        lastIndex = dType.SetIOPWM(self.api, 4, 5, 0.75,isQueued=1)[0]
        lastIndex = dType.SetWAITCmd(self.api, 1, isQueued=1)[0]
        # 回位
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 62, 0, 0, 0, isQueued = 1)[0]
        
        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True

    # 机械臂指位置，receptionist用
    def dobot_point(self):
        rospy.loginfo("point")

        #清空队列
        dType.SetQueuedCmdClear(self.api)
        
        #设置运动参数
        dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued = 1)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)

        #开始执行指令队列
        dType.SetQueuedCmdStartExec(self.api)

        #设置ptpcmd内容并将命令发送给dobot
        result = dType.SetCmdTimeout(self.api, 100)
        dType.SetEndEffectorParams(self.api,0,0,0,isQueued=0)
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 250, 0, 50, 0, isQueued = 1)[0]
        lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, 150, 0, 50, 0, isQueued = 1)[0]

        #如果还未完成指令队列则等待
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(10)

        print('timeout:', result)

        #停止执行指令
        dType.SetQueuedCmdStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        if result != None:
            return False
        else:
            return True
        
    def dobot_main(self,data):
        goal = data.goal
        print('goal is to',goal)
        if goal == 'abort':
            dType.SetQueuedCmdForceStopExec(self.api)
            self.dobot_disconnect()
        if goal == 'reset': # reset后执行其他动作会触发段错误卡死
            state = self.dobot_reset()
            return state
        
        if goal == 'point':
            state = self.dobot_point()
            return state
        # storing groceries机械臂抓取1
        if goal == 'groceries_catch1':
            state = self.dobot_groceries_catch1()
            return state
        # storing groceries机械臂抓取2
        if goal == 'groceries_catch2':
            state = self.dobot_groceries_catch2()
            return state
                # storing groceries机械臂放置
        if goal == 'groceries_place':
            state = self.dobot_place()
            return state
        # # storing groceries机械臂放置1
        # if goal == 'groceries_place1':
        #     state = self.dobot_place1()
        #     return state
        # # storing groceries机械臂放置2
        # if goal == 'groceries_place2':
        #     state = self.dobot_place2()
        #     state = self.dobot_reset()
        #     return state
        # carry my luggage机械臂抓取1
        if goal == 'carry_catch1':
            state = self.dobot_carry_catch1()
            return state
        # carry my luggage机械臂抓取2
        if goal == 'carry_catch2':
            state = self.dobot_carry_catch2()
            return state
        # carry my luggage机械臂给予1
        if goal == 'carry_give1':
            state = self.dobot_carry_give1()
            return state
        # carry my luggage机械臂给予2
        if goal == 'carry_give2':
            state = self.dobot_carry_give2()
            return state
        # carry my luggage机械臂给予1
        if goal == 'gpsr_give1':
            state = self.dobot_gpsr_give1()
            return state
        # carry my luggage机械臂给予2
        if goal == 'gpsr_give2':
            state = self.dobot_gpsr_give2()
            return state
        if goal == 'drop':
            state = self.dobot_drop()
            state = self.dobot_turn()
            return state
        if goal == 'reach':
            state = self.dobot_reach()
            return state
        if goal == 'move':
            state = self.dobot_move()
            return state
        if goal == 'back':
            state = self.dobot_back()
            return state
        if goal == 'turn':
            state = self.dobot_turn()
            return state
        if goal == 'clear':
            self.dobot_clear()
            return True


if __name__ == '__main__':
    rospy.init_node('dobot_server',anonymous=True)
    rospy.Rate(10)
    dobot = DobotControl()
    if (dobot.state != dType.DobotConnect.DobotConnect_NoError):
        dType.DisconnectDobot(dobot.api)
        rospy.logerr('cannot connect dobot')
    else:   
        rospy.loginfo('connect dobot successful')
        s = rospy.Service("dobot_goal", dobot_control, dobot.dobot_main)
    rospy.spin()
