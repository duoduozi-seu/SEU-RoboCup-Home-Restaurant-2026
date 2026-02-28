#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import DobotDllType as dType
import math
from ctypes import c_byte
# import dobot_server as dserver

def dobot_turn():

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
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, -16, -120, 30, 0, isQueued=1)[0]

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

def dobot_move():
    # 清空队列
    dType.SetQueuedCmdClear(api)
    dt = 0.70

    # 设置运动参数
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)

    # 开始执行指令队列
    dType.SetQueuedCmdStartExec(api)

    # 设置ptpcmd内容并将命令发送给dobot
    result = dType.SetCmdTimeout(api, 100)
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    dobotpt = [-169, -86]
    ptlen = math.sqrt(dobotpt[0] ** 2 + dobotpt[1] ** 2)
    # if ptlen > 260:
    #     print('out of range')
    #     dobotpt[0] = int(dobotpt[0] * 260 / ptlen)
    #     dobotpt[1] = int(dobotpt[1] * 260 / ptlen)
    # print("++++++++++++", dobotpt[0], dobotpt[1])

    # 盲伸位置
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, 0, 0, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 2, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, 10, 0, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 2, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, 20, 0, isQueued=1)[0]
    #
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, 0.3, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 2, isQueued=1)[0]
    # #完成第一个物品放置
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 230, 0, 0, 41.108821868896484, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 186, -4.316, -30, 41.108821868896484, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 206, -4.316, -30, 41.108821868896484, isQueued=1)[0]



    # 闭合手爪
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, dt, isQueued=1)[0]


    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 120, 0, 30, 0, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 192, 0, -125, 0, isQueued=1)[0]
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

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}


#将dll读取到内存中并获取对应的CDLL实例
#Load Dll and get the CDLL object
api = dType.load()
#建立与dobot的连接
#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):
    #清空队列
    #Clean Command Queued
    dType.SetQueuedCmdClear(api)

    #设置运动参数
    #Async Motion Params Setting
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPJointParams(api, 150, 150, 150, 150, 150, 150, 150, 150, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)

    #回零
    #Async Home
    # dType.SetHOMECmd(api, temp = 0, isQueued = 1)
    # print("Finish Async Home!")
    pos = dType.GetPose(api)
    print(pos, "-------")

    dType.dSleep(2000)
    # 设置ptpcmd内容并将命令发送给dobot
    dobot_turn()
    dobot_move()

# 断开连接
# Disconnect Dobot
dType.DisconnectDobot(api)

