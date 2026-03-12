#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import sys
import time
from pathlib import Path

import smach
import smach_ros
COMMON_ROOT = Path(__file__).resolve().parents[2] / "common"
if str(COMMON_ROOT) not in sys.path:
    sys.path.insert(0, str(COMMON_ROOT))
from common import *
import requests
import json
import re
import rospy
import math
import datetime
from seu_info_inform.srv import *
weekdays = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
def get_info(key):
    try:
        result = rospy.get_param(key)
    except:
        result = None
    return result

# 语音识别
def srv_speech_recognition(task = "none"):
    srv_name = 'speech_recognition_service'  # 当前服务的名称
    client = rospy.ServiceProxy(srv_name, seu_speech_recognition)  # 当前操作的服务
    rospy.wait_for_service(srv_name)  # 等待服务
    rospy.loginfo(srv_name + ' available')
    try:
        request = seu_speech_recognitionRequest()
        request.task = task
        request.enable = False  # 请求值，加前缀防止重名
        response = client.call(request)  # 返回值
        if response.state == True:
            rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
            return response.word
        else:
            rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
            rospy.logerr("errormsg: " + response.errormsg)
            return -1  # 调用失败，返回-1

    except rospy.ServiceException as e:  # 调用失败，输出具体信息
        print("Service call Failed: " + srv_name + ": %s" % e)
        return -1
    

# 语音输入
class SpeechState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             output_keys=['speech_output'])  # 状态输出数据speech_output，即用户语音输入后的识别内容

    def execute(self, userdata):
        # 执行操作

        word = srv_speech_recognition("answer")  # 调用语音识别服务

        # word = 'navigate to the living room,meet Robin,and follow him' #测试用
        if word != -1:
            userdata.speech_output = word
            rospy.loginfo('word: ' + word)
            return 'success'
        else:
            return 'failure'


# 确认输入是否正确
class VerifyState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             input_keys=['verify_input'],  # 状态输入数据verify_input，即上一状态（语音输入）的识别内容
                             output_keys=['verify_output'])  # 状态输出数据verify_output，即用于分析的用户输入指令

    def execute(self, userdata):
        # 执行操作
        str = userdata.verify_input
        verifystr = "the question is: " + str + ", is that right?"
        # 语音播报
        speech_to_text(verifystr)
        rospy.loginfo('%s', verifystr)
        result = self.verify()

        if result:
            userdata.verify_output = str
            return 'success'
        else:
            return 'failure'

    def verify(self):
        result = srv_speech_recognition()  # 调用语音识别服务
        if result == -1:  # 识别为空，返回错误
            return False
        else:
            rospy.loginfo('word: %s', result)
            if 'yes' in result or 'ok' in result:  # 用户回答 “yes,you are right",即为正确，否则为识别有误
                return True
            else:
                if 'right' in result:
                    return True
                else:
                    return False

class AnswerState(smach.State):
    def __init__(self,arg):
        smach.State.__init__(self,
                            outcomes=['success', 'failure'],
                            input_keys=['answer_input'])# 状态输入数据，即需要解答的问题，由语音识别得到
        self.prompt = arg

    def execute(self,userdata):
        # 执行操作
        question = userdata.answer_input
        rospy.loginfo("The question is: %s",question)
        result = self.answer(question)
        if result:
            return 'success'
        else:
            return 'failure'

    def answer(self,question):# 调用大模型服务解答问题
        srv_name = "gemini_llm_service" # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_llm)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_llmRequest()
            request.task = 'answer'
            request.command = f"{self.prompt},the question is {question}"
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                rospy.loginfo('The answer is : %s',response.answer)
                speech_to_text(response.answer) # 语音播报问题答案
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

class InfoInformService:
    def __init__(self):

        self.person_count = 0
        self.today_date = datetime.date.today().strftime("%Y-%m-%d")
        self.current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        self.team_name = "southest at home"
        self.team_country = "China"
        self.team_affiliation = "Southeast university"
        self.day_of_month = str(datetime.date.today().day)
        self.day_of_week = weekdays[datetime.datetime.now().weekday()]
        self.other = ""
        self.ori_command = ""
        self.obj_p = "biggest: water>cola=sprite>dishsoap>chip>lays>biscuit>cookie>handwash>shmpoo>bread； \
                      heaviest： water>cola=sprite>handwash>shampoo>biscuit>chip=lays>bread>cookie；\
                      thinest: lays>cola>sprite>water>dishsoap>chip>bread>biscuit>handwash>cookie>shampoo"


    def get_prompt(self):
        return f"Some known information as follows, today date is {self.today_date},current time is {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}，our team name is {self.team_name},team country is {self.team_country},team affiliation is {self.team_affiliation},the day pf the mounth is {self.day_of_month},the day of the week is {self.day_of_week},{self.other}"

    def inform(self,command):
        srv_name = "gemini_llm_service"  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_llm)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        srv_response = seu_info_informResponse()
        try:
            request = seu_llmRequest()
            request.task = 'answer'
            request.command = f"{self.get_prompt()},somethong about object {self.obj_p},the ori command is {command},now i will answer to the target people,what should i answer"
            print(request.command)
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                rospy.loginfo('The answer is : %s', response.answer)
                speech_to_text(response.answer)  # 语音播报问题答案
                srv_response.errormsg = 'success'
                srv_response.state = True
                return srv_response
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                srv_response.errormsg = 'fail'
                srv_response.state = False
                return srv_response
        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            srv_response.errormsg = 'fail'
            srv_response.state = False
            return srv_response
        
    def answer(self,arg):
        sm = smach.StateMachine(outcomes=['answer_success', 'answer_failure'])

        # 根据列表中的动作，添加不同的动作状态
        with sm:
            smach.StateMachine.add('Speech_answer', SpeechState(),  # 首先语音输入问题
                                   transitions={'success': 'Verify_answer',
                                                'failure': 'Speech_answer'})
            smach.StateMachine.add('Verify_answer', VerifyState(),  # 确认输入是否正确
                                   transitions={'success': 'Answer',
                                                'failure': 'Speech_answer'},
                                   remapping={'verify_input': 'speech_output'})
            smach.StateMachine.add('Answer', AnswerState(arg),  # 大模型解答问题
                                   transitions={'success': 'answer_success',
                                                'failure': 'answer_failure'},
                                   remapping={'answer_input': 'verify_output'})
        result = sm.execute()
        rospy.loginfo("result:%s", result)
        # 根据状态机运行结果应答
        response = seu_info_informResponse()
        if result == 'answer_success':
            response.errormsg = 'success'
            response.state = True
        else:
            response.errormsg = 'fail'
            response.state = False

        return response

    def callback(self,req):
        if req.info_kind =="person_count":
            self.other = f"the target person counted is {get_info('person_count')}"
        elif req.info_kind == "gesture":
            self.other = f"the target people gesture is {get_info('gesture')},"
        elif req.info_kind == "gender":
            self.other = f"the target people gender is {get_info('gender')},"
        elif req.info_kind == "obj_name":
            self.other = f"the target obj is {get_info('obj_name')}"
        elif req.info_kind == "obj_count":
            self.other = f"the target obj is counted {get_info('obj_count')}"
        elif req.info_kind == "answer":
            response = self.answer(self.get_prompt())
            #speech_to_text("good bye, i will to do next thing")
            return response
        elif req.info_kind == "introduce":
            person_info = get_info('person_info')
            if person_info is not None:
                self.other = f"the target person is {person_info}"

        response = self.inform(req.ori_command)
        #speech_to_text("good bye, i will to do next thing")
        return response
if __name__ == "__main__":
    rospy.init_node('info_inform_node')
    info_inform = InfoInformService()
    server = rospy.Service("info_inform_service", seu_info_inform, info_inform.callback)
    rospy.loginfo("info inform service ready")
    #循环接收消息
    rospy.spin()
