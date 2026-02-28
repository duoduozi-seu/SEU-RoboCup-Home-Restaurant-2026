#! /usr/bin/env python
import numpy as np
import pyttsx3
import rospy
import DobotDllType as dType
from robot_msgs.msg import object_position
import math

item_name = 'item'
item_name_tmp = 'item'
item_count = -1
count = 0
p =object_position()

def speech_to_text(data):
    speech=pyttsx3.init()
    voices=speech.getProperty('voices')
    speech.setProperty('voice',voices[10].id)
    speech.setProperty('rate',160)
    speech.say(data)
    speech.runAndWait()

def dobot_calculate():
    global p
    # 手眼标定矩阵
    image_to_arm=         [[-4.16328058e+01, -7.97633356e+02,  5.20538611e+02, -1.60018363e+02],
                             [-4.55749074e+02,  1.72453374e+02, -1.70390317e+02,  8.40024099e+01],
                             [-1.12492154e+01, -7.20126757e+02, -6.34552436e+02,  5.88034475e+02],
                             [ 8.88178420e-16,  3.55271368e-15,  1.77635684e-15,  1.00000000e+00]]
    pt=[p.x, p.y, p.z,1.0]
    dobotpt=np.dot(image_to_arm, np.array(pt))[0:3]
    return dobotpt

def arm_callpack(msg):
    print('444444444444444444444')
    global item_name, item_name_tmp, item_count, count, p

    item_name_tmp = msg.label
    p.label = msg.label
    p.x = msg.x
    p.y = msg.y
    p.z = msg.z

    if item_count == -1:
        item_name = item_name_tmp
        item_count += 1
    if item_name != msg:
        item_count += 1
    if item_count > 5:
        item_name = item_name_tmp
        count += 1
        item_count = -1
    if count > 7 :
        dobot_catch()
        count = 0

# 机械臂转到一侧去
def dobot_turn():
    rospy.loginfo("turn")
    # 清空队列
    dType.SetQueuedCmdClear(api)

    # 设置运动参数
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)

    # 开始执行指令队列
    dType.SetQueuedCmdStartExec(api)

    # 设置ptpcmd内容并将命令发送给dobot
    result = dType.SetCmdTimeout(api, 100)

    # 机械臂转到一侧，防止遮挡物体，影响识别
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 3, -106, -4, 0, isQueued=1)[0]

    # 如果还未完成指令队列则等待
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(10)

    print('timeout:', result)

    # 停止执行指令
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    if result != None:
        return False
    else:
        return True

# 机械臂抓取
def dobot_catch():
    if item_name == 'chip' or item_name == 'biscuit':
        dC = 0.7
    elif item_name == 'sprite' or item_name == 'water' or item_name == 'cola' or item_name == 'lays' or item_name == 'orange juice':
        dC = 0.72
    elif item_name == 'dishsoap':
        dC = 0.73
    elif item_name == 'handwash':
        dC = 0.66
    else:
        dC = 0.75
    rospy.loginfo("catch")
    dobotpt = dobot_calculate()
    rospy.loginfo(dobotpt)
    # speech_to_text('to catch the item')
    # 判断是否超限

    ptlen = math.sqrt(dobotpt[0] ** 2 + dobotpt[1] ** 2)
    if ptlen > 240:
        speech_to_text('out of range')
        dobot_move()
        return
    # 清空队列
    dType.SetQueuedCmdClear(api)

    # 设置运动参数
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPJointParams(api, 150, 150, 150, 150, 150, 150, 150, 150, isQueued=1)
    dType.SetPTPCommonParams(api, 75, 75, isQueued=1)

    # 开始执行指令队列
    dType.SetQueuedCmdStartExec(api)

    # 设置ptpcmd内容并将命令发送给dobot
    result = dType.SetCmdTimeout(api, 100)
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 3, -106, 40, 0, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    # 张开手爪
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, 0.3, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    # 伸向物体
    dType.SetEndEffectorParams(api, 0, 0, 30, isQueued=0)
    # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 133, -9, 15, 0, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    # lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, dobotpt[0], dobotpt[1], dobotpt[2], 0, isQueued=1)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, dobotpt[0], dobotpt[1], -10, 0, isQueued=1)

    dType.SetPTPJointParams(api, 25, 25, 25, 25, 25, 25, 25, 25, isQueued=1)
    dType.SetPTPCommonParams(api, 10, 10, isQueued=1)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, dobotpt[0]+15, dobotpt[1], -10, 0, isQueued=1)

    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    # 闭合手爪
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, dC, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    # 稍微上升手爪
    # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, dobotpt[0]+10, dobotpt[1], 30, 0, isQueued=1)
    lastIndex = dType.SetWAITCmd(api, 1.5, isQueued=1)[0]

    # 回位
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 100, 0, 0, 0, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 150, -5, 40, 0, isQueued=1)[0]

    # 如果还未完成指令队列则等待
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(10)

    print('timeout:', result)

    # 停止执行指令
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)

    if result != None:
        return False
    else:
        return True
    exit()

def dobot_move():
    global item_name
    if item_name == 'chip' or item_name == 'biscuit':
        dC = 0.7
    elif item_name == 'sprite' or item_name == 'water' or item_name == 'cola' or item_name == 'lays' or item_name == 'orange juice':
        dC = 0.72
    elif item_name == 'dishsoap':
        dC = 0.73
    elif item_name == 'handwash':
        dC = 0.66
    else:
        dC = 0.75
    rospy.loginfo("move")
    rospy.loginfo("盲抓")
    # 清空队列
    dType.SetQueuedCmdClear(api)

    # 设置运动参数
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)

    # 开始执行指令队列
    dType.SetQueuedCmdStartExec(api)

    # 设置ptpcmd内容并将命令发送给dobot
    result = dType.SetCmdTimeout(api, 100)
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 100, 0, 0, 0, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]
    # 张开手爪

    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, 0.3, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    # 盲伸位置
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 231.823, 2.999, 8.718, 0, isQueued=1)[0]
    rospy.loginfo('robot state is:{}'.format(dType.GetPose(api)))
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    speech_to_text('please help me catch the')
    speech_to_text(item_name)
    flag = 0
    lastIndex = dType.SetWAITCmd(api, 2, isQueued=1)[0]

    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, dC, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, 30, 0, isQueued=1)[0]

    # 如果还未完成指令队列则等待
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(10)

    print('timeout:', result)

    # 停止执行指令
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    if result != None:
        return False
    else:
        return True
    
def dobot_aimed_catch():
    pass

if __name__ == '__main__':
    # 机械臂初始化
    CON_STR = {
        dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
        dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
        dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

    api = dType.load()
    state = dType.ConnectDobot(api, "", 115200)[0]
    rospy.loginfo("Connect status:{}".format(CON_STR[state]))
    if (state != dType.DobotConnect.DobotConnect_NoError):
        print('error')
    rospy.loginfo('robot state is:{}'.format(dType.GetPose(api)))

    # test code
    dobot_turn()

    rospy.init_node('dobot', anonymous=True)
    sub = rospy.Subscriber("/detect_result2", object_position, arm_callpack, queue_size=10)
    rospy.spin()


