#! /home/syw/anaconda3/envs/GPSR/bin/python
from datetime import datetime
import os
import rospy
import smach
import smach_ros
from sympy import false

from seu_smach_main.srv import *
import sys
import pyttsx3
import time
from fpdf import FPDF
from robot_msgs.msg import object_position
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from dobot.srv import dobot_control
sys.path.append("/home/syw/code/robocup_nation/2024code/GPSR/src")
from common.camera import Camera

# 服务名称
seuhome_speech_recognition_name = 'speech_recognition_service'
seuhome_llm_name = 'baidu_llm_service'
seuhome_find_person_name = 'find_person_service'
seuhome_find_object_name = 'find_object_service'
seuhome_nav_name = 'nav_service'
seuhome_follow_name = 'follow_service'
seuhome_grasp_name = "dobot_server"
find_person_count=0
find_object_count=0
# 语音播报状态，分input和answer两种
speech_state = 'input'
ori_command = ""
debug = False
def get_info(key):
    try:
        result = rospy.get_param(key)
    except:
        result = None
    return result

def rotate_robot(vel_pub, angular_speed, duration):
    """封装机器人旋转动作"""
    rate = rospy.Rate(10)
    end_time = rospy.Time.now().to_sec() + duration
    while rospy.Time.now().to_sec() < end_time:
        vel_cmd = Twist()
        vel_cmd.angular.z = angular_speed
        vel_pub.publish(vel_cmd)
        rate.sleep()
    # 停止旋转
    vel_pub.publish(Twist())

from fpdf import FPDF

def txt_to_pdf_fpdf(input_txt, output_pdf):
     # 确保日志目录存在
    log_dir = os.path.dirname(input_txt)
    if log_dir and not os.path.exists(input_txt):
        return
    # 创建 PDF 对象
    pdf = FPDF()
    pdf.add_page()  # 添加新页面
    pdf.set_font("Arial", size=12)  # 设置字体
    # 读取 TXT 文件并写入 PDF
    with open(input_txt, "r", encoding="utf-8") as file:
        for line in file:
            pdf.cell(0, 10, txt=line.strip(), ln=True)  # 每行写入 PDF

    # 保存 PDF
    pdf.output(output_pdf)

# 定义输入部分状态类
# 语音输入
class SpeechState(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['success', 'failure'],
                            output_keys=['speech_output'])# 状态输出数据speech_output，即用户语音输入后的识别内容

    def execute(self, userdata):
        # 执行操作
        # 不同任务提示语不同
        if speech_state == 'input':
             # 语音识别
            if False:
                word = input("please enter command")
            else:
                word = srv_speech_recognition("command") # 调用语音识别服务
                if word =="input":
                    word = input("please enter command")
        elif speech_state == 'answer':
            word = srv_speech_recognition("answer") # 调用语音识别服务
       
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
                             input_keys=['verify_input'],# 状态输入数据verify_input，即上一状态（语音输入）的识别内容
                             output_keys=['verify_output'])# 状态输出数据verify_output，即用于分析的用户输入指令

    def execute(self, userdata):

        # 执行操作
        str = userdata.verify_input

        userdata.verify_output = str
        # if debug:
        #     return 'success'

        if speech_state == 'input':
            verifystr = "the command is: " + str + ", is that right?"
        elif speech_state == 'answer':
            verifystr = "the question is: " + str + ", is that right?"
        # 语音播报
        speech_to_text(verifystr)
        rospy.loginfo('%s',verifystr)
        result = self.verify()

        if result:
            userdata.verify_output = str
            return 'success'
        else:
            return 'failure'

    def verify(self):
        result = srv_speech_recognition() # 调用语音识别服务
        if result == -1:# 识别为空，返回错误
            return False
        else:
            rospy.loginfo('word: %s',result)
            if 'yes' in result or 'ok' in result:# 用户回答 “yes,you are right",即为正确，否则为识别有误
                return True
            else:
                if 'right' in result:
                    return True
                else:
                    return False
                
# 大模型分析指令
class AnalyzeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success', 'failure'],
                             input_keys=['analyze_input'],# 状态输入数据analyze_input，即为语音输入的指令
                             output_keys=['analyze_output'])# 状态输出数据analyze_output，即为模型分析后的python list

    def execute(self, userdata):
        # 执行操作
        word = "Analyzing the command"
        # 语音播报
        speech_to_text(word)
        analyze_str = userdata.analyze_input
        rospy.loginfo("Task: %s",analyze_str)
        result = self.analyze(userdata)
        if result:
            return 'success'
        else:
            userdata.analyze_output = []
            return 'failure'

    def analyze(self, userdata):# 调用大模型服务解析指令
        srv_name = seuhome_llm_name  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_llm)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_llmRequest()
            request.task = 'analyze'
            request.command = userdata.analyze_input  
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                action_lst = eval(response.answer)
                rospy.loginfo('Analyze result: %s',response.answer)
                userdata.analyze_output = action_lst
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False  

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

# 定义动作基元状态类
# 导航
class MoveToState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0].replace('_',' ') # 参数，即导航目的地

    def execute(self,userdata):
        # 执行操作
        word = "Moving to "+ str(self.target)
        # 语音播报
        speech_to_text(word)
        # if self.target == 'initial location':  # 测试用
        #     return 'success' 
        # if debug:
        #     return 'success'
        word="Successfully arrived at"+ str(self.target)
        result = self.move_to(self.target)
        if result:
            speech_to_text(word)
            return 'success'
        else:
            return 'failure'

    def move_to(self, target):# 调用导航服务
        rospy.loginfo("Moving to {}".format(target))
        srv_name = seuhome_nav_name  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_nav)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_navRequest()
            request.des = target
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False  

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

# 抓取
class GraspState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.detect_result_pub = rospy.Publisher(
            '/detect_result2', object_position, queue_size=10
        )
        self.cli = rospy.ServiceProxy("dobot_goal", dobot_control)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.msg = String()
        self.obj = arg[0]

    def execute(self, userdata):
        pos = get_info("obj_pos")
        if pos is not None:
            p = object_position()
            p.label =pos['label']
            p.x = pos['x']
            p.y = pos['y']
            p.z = pos['z']
            self.detect_result_pub.publish(p)
            # 使用状态内部的发布器和消息对象
            rospy.loginfo("机械臂正在向前抓取...")
            speech_to_text(f"start to grasp {self.obj}")
            self.req = dobot_control._request_class()
            self.req.goal = "groceries_catch1"
            reqr = self.cli(self.req)

            rospy.loginfo("机器人正在后退...")
            vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # 根据实际情况调整
            twist_msg = Twist()
            twist_msg.linear.x = -1  # 缓慢后退
            vel_pub.publish(twist_msg)
            rospy.sleep(5)  # 前进一段时间
            rotate_robot(self.vel_pub, angular_speed=-0.5, duration=7)

            rospy.loginfo("机械臂正在向后收缩...")
            self.req = dobot_control._request_class()
            self.req.goal = "groceries_catch2"
            reqr = self.cli(self.req)
            speech_to_text("finish grasp")
            rospy.loginfo("机械臂抓取成功")
            return 'success'
        else:
            return 'failure'



# 传递
class DeliverState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0]
        self.cli = rospy.ServiceProxy("dobot_goal", dobot_control)
        self.msg = String()
        self.word = ""
        rospy.Subscriber('/real_time_speech_recognition_topic', String, self.speech_recognition_callback)

    def execute(self, userdata):
        # 语音播报
        speech_to_text( f"Passing to {self.target}")
        self.req = dobot_control._request_class()
        self.req.goal = "gpsr_give1"
        reqr = self.cli(self.req)

        speech_to_text("please say ok after you get it")
        rospy.set_param("real_time_speech_recognition_enable", True)
        last_time = time.time()
        times = 0
        while True:
            if 'yes' in self.word or 'ok' in self.word or 'right' in self.word or 'stop' in self.word or times >2:
                rospy.set_param("real_time_speech_recognition_enable", False)
                break
            if time.time() - last_time > 5:
                last_time = time.time()
                speech_to_text("please get it away and answer ok")
                times = times + 1

        rospy.loginfo("机械臂正在向后收缩...")
        self.req = dobot_control._request_class()
        self.req.goal = "gpsr_give2"
        reqr = self.cli(self.req)
        return 'success'

    def speech_recognition_callback(self,msg):
        if msg.data != "":
            self.word = msg.data

# 放置
class PutOnState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.obj = arg[0]
        self.target = arg[1]
        self.cli = rospy.ServiceProxy("dobot_goal", dobot_control)
        self.msg = String()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def execute(self, userdata):
        # 语音播报
        speech_to_text(f"Put {self.obj} to {self.target}")
        result = self.pass_to(self.target)
        if result:
            return 'success'
        else:
            return 'failure'

    def pass_to(self, target): # 实际功能尚未编写
        twist_msg = Twist()
        rospy.loginfo("正在放置物体...")
        self.req = dobot_control._request_class()
        self.req.goal = "groceries_place"
        reqr = self.cli(self.req)

        # twist_msg.linear.x = 0.2  # 前进
        # self.vel_pub.publish(twist_msg)
        # rospy.sleep(1)  # 一段时间

        # self.req.goal = "groceries_place2"
        # reqr = self.cli(self.req)
        # rospy.loginfo("物体放置成功")
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # 根据实际情况调整
        twist_msg.linear.x = -1  # 后退
        vel_pub.publish(twist_msg)
        rospy.sleep(4)  # 后退一段时间
        # rotate_robot(self.vel_pub, angular_speed=-0.5, duration=7)
        return True

# 找人    
class LookForPersonState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0] # 参数，即所找人名字或特征
        self.count = arg[1]

    def execute(self,userdata):
        # 执行操作
        # if debug:
        #     return 'success'
        word = "Looking for person "+ str(self.target)
        # 语音播报
        speech_to_text(word)
        result = self.look_for_person(self.target)



        #word= "Are you a person"+str(self.target)+"?"
        #speech_to_text(word)
        #listen = srv_speech_recognition() # 调用语音识别服务
        #if listen == -1:
            #return "failure"
        #else:
            #if 'no' in result:
                #return "failure"


        word= "Successfully found person" +str(self.target)
        if result:
            speech_to_text(word)
            return 'success'
        else:
            return 'failure'

    def look_for_person(self, target): # 调用找人服务
        rospy.loginfo("Looking for person {}".format(target))
        srv_name = seuhome_find_person_name  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_findperson)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_findpersonRequest()
            request.feature = target
            request.count = self.count
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False  

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

# 人物分析
class PersonAnalyzeState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.feature = arg[0]

    def execute(self, userdata):
        # 执行操作
        word = "Start analyse people"
        # 语音播报
        speech_to_text(word,speak=false)

        result = self.person_analyse(self.feature)
        word = "Successfully analyse person" + str(self.feature)
        if result:
            speech_to_text(word,speak=false)
            return 'success'
        else:
            return 'failure'

    def person_analyse(self,feature): # 调用找物服务
        rospy.loginfo("Looking for object {}".format(feature))
        srv_name = "person_analyse_service"  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_person_analyse)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_person_analyseRequest()
            request.feature = feature
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

# 物品分析
# 该功能合并至find_obj
class ObjAnalyzeState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0]

    def execute(self, userdata):
        return 'success'



# 物品分析
# 该功能合并至find_obj
class ObjCountState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0]

    def execute(self, userdata):
        return 'success'

# 找物
class LookForObjState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0] # 参数，即所找物品名称
        self.feature = arg[1]

    def execute(self,userdata):
        # 执行操作
        word = "Looking for object "+ str(self.target)
        # 语音播报
        speech_to_text(word)
        
        result = self.look_for_obj(self.target)
        word= "Successfully found object" + str(self.target)
        if result:
            speech_to_text(word)
            return 'success'
        else:
            return 'failure'

    def look_for_obj(self, target): # 调用找物服务
        rospy.loginfo("Looking for object {}".format(target))
        srv_name = seuhome_find_object_name  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_findobject)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_findobjectRequest()
            request.object = target
            request.feature = self.feature
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False  

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

# 跟随 
class FollowState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.location = arg[0]
        self.target = "person" # 参数，即跟随人名称，实际不起作用

    def execute(self,userdata):
        # 执行操作
        word = "Following "+ str(self.target)
        # 语音播报
        speech_to_text(word)
        result = self.follow(self.target)
        word="Successful followed" + str(self.target)
        if result:
            speech_to_text(word)
            return 'success'
        else:
            return 'failure'

    def follow(self, target):# 调用跟随服务
        rospy.loginfo("Following {}".format(target))
        srv_name = seuhome_follow_name  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_follow)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_followRequest()
            request.person = target
            request.location = self.location
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False  

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

# 引导
class LeadToState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0]

    def execute(self, userdata):
        rospy.loginfo(f"Lead to {self.target}")
        speech_to_text(f"Lead to {self.target}")
        srv_name = "lead_service"  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_lead)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_leadRequest()
            request.location = self.target
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                return 'success'
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return 'failure'

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return 'failure'


# 语音交流          
class SpeakState(smach.State):
    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0]

    def execute(self,userdata):
        # 执行操作
        result = self.speak(self.target)
        if result:
            return 'success'
        else:
            return 'failure'

    def speak(self, target): # 调用语音播报，播报内容一般由大模型在解析阶段给出
        speech_to_text(target)
        return True

# 回答问题    
class AnswerState(smach.State):
    def __init__(self,arg):
        # smach.State.__init__(self, 
        #                     outcomes=['success', 'failure'],
        #                     input_keys=['answer_input'])# 状态输入数据，即需要解答的问题，由语音识别得到
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.question = arg[0]

    def execute(self,userdata):
        # 执行操作
        # question = userdata.answer_input
        rospy.loginfo("The question is: %s",self.question)
        result = self.answer(self.question)
        if result:
            return 'success'
        else:
            return 'failure'

    def answer(self,question):# 调用大模型服务解答问题
        srv_name = seuhome_llm_name  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_llm)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_llmRequest()
            request.task = 'answer'
            request.command = question  
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

# 告知信息
class InfoInformState(smach.State):
    def __init__(self,arg):
        smach.State.__init__(self, 
                            outcomes=['success', 'failure'],
                            input_keys=['answer_input'])# 状态输入数据，即需要解答的问题，由语音识别得到
        self.info_kind = arg[0]

    def execute(self,userdata):
        global ori_command
        # 执行操作
        question = ori_command
        rospy.loginfo("The question is: %s",question)
        result = self.answer(question)
        if result:
            return 'success'
        else:
            return 'failure'

    def answer(self,question):# 调用大模型服务解答问题
        srv_name = "info_inform_service"  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_info_inform)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_info_informRequest()
            request.info_kind = self.info_kind
            request.ori_command = question  
            response = client.call(request)  # 返回值
            if response.state == True:
                rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
                return True
            else:
                rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
                rospy.logerr("errormsg: " + response.errormsg)
                return False  

        except rospy.ServiceException as e:  # 调用失败，输出具体信息
            print("Service call Failed: " + srv_name + ": %s" % e)
            return False

# 语音合成
def speech_to_text(data, log_file= None,speak = True):
    try:
        if speak:
            # 语音合成
            speech = pyttsx3.init()
            voices = speech.getProperty('voices')
            speech.setProperty('voice', voices[2].id)
            speech.setProperty('rate', 140)
            speech.say(data)
            speech.runAndWait()
        if log_file is None:
            log_file = get_info('log')
        # 获取当前时间戳
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # 确保日志目录存在
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        # 追加写入日志文件
        with open(f"{log_file}.txt", "a", encoding="utf-8") as f:
            f.write(f"[{timestamp}] {data}\n")
    except Exception as e:
        print(f"语音合成或日志记录出错: {e}")

# 语音识别
def srv_speech_recognition(task = "none"):
    srv_name = seuhome_speech_recognition_name  # 当前服务的名称
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

# 创建执行状态机
def create_state_machine(actions,ori_command):
    # 将列表中的动作,参数和序号拼接，作为每个状态的名称
    state_names = [f'{action[0]} {action[1:]} {index}' for index, action in enumerate(actions)]
    state_names.append('sub_success') # 在末端添加‘sub_success’，作为最后一个状态的状态转移

    # 创建状态机
    sm = smach.StateMachine(outcomes=['sub_success', 'sub_failure'])
    sm.userdata.ori_command = ori_command
    # 根据列表中的动作，添加不同的动作状态
    with sm:
        for i in range(len(actions)):
            action = actions[i][0]
            arg = actions[i][1:]
            if action == 'move_to':
                smach.StateMachine.add(state_names[i],
                                       MoveToState(arg),
                                       transitions={'success': state_names[i+1],# 当前状态成功，跳转下一状态，否则重复当前状态。下同
                                                    'failure': state_names[i]})
            elif action == 'grasp':
                smach.StateMachine.add(state_names[i],
                                       GraspState(arg),
                                       transitions={'success': state_names[i+1],# grasp和pass_to具体逻辑代码未编写
                                                    'failure': state_names[i]})
            elif action == 'deliver':
                smach.StateMachine.add(state_names[i],
                                       DeliverState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'put_on':
                smach.StateMachine.add(state_names[i],
                                       PutOnState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'obj_analyze':
                smach.StateMachine.add(state_names[i],
                                       ObjAnalyzeState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'obj_count':
                smach.StateMachine.add(state_names[i],
                                       ObjCountState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'person_analyze':
                smach.StateMachine.add(state_names[i],
                                       PersonAnalyzeState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'info_inform':
                smach.StateMachine.add(state_names[i],
                                       InfoInformState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'look_for_person':
                smach.StateMachine.add(state_names[i],
                                       LookForPersonState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'look_for_obj':
                smach.StateMachine.add(state_names[i],
                                       LookForObjState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'follow':
                smach.StateMachine.add(state_names[i],
                                       FollowState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'lead_to':
                if arg[0] == "none":
                    speech_to_text("sorry,you dont tell me lead the people to where")
                    return None

                smach.StateMachine.add(state_names[i],
                                       LeadToState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'speak':
                smach.StateMachine.add(state_names[i],
                                       SpeakState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]})
            elif action == 'answer':
                smach.StateMachine.add(state_names[i],
                                       AnswerState(arg), # 如果是要回答问题，创建回答问题的嵌套状态机
                                       transitions={'success': state_names[i+1],
                                                    'failure': 'sub_failure'})

    return sm

# 创建回答问题状态机
def create_answer_smach():
    global speech_state
    speech_state = 'answer'
    # 创建状态机
    sm = smach.StateMachine(outcomes=['answer_success', 'answer_failure'])
    if sm is None:
        return None
    # 根据列表中的动作，添加不同的动作状态
    with sm:
        smach.StateMachine.add('Speech_answer', SpeechState(), # 首先语音输入问题
                                transitions={'success':'Verify_answer', 
                                             'failure':'Speech_answer'})
        smach.StateMachine.add('Verify_answer', VerifyState(), # 确认输入是否正确
                                transitions={'success':'Answer', 
                                             'failure':'Speech_answer'},
                                remapping={'verify_input': 'speech_output'})
        smach.StateMachine.add('Answer', AnswerState(), # 大模型解答问题
                                transitions={'success':'answer_success', 
                                             'failure':'answer_failure'},
                                remapping={'answer_input': 'verify_output'})

    return sm

if __name__ == '__main__':
    rospy.init_node('gpsr_smach_node')
    find_person_count=0
    find_object_count=0
    log = f'/home/syw/code/robocup_nation/2024code/GPSR/src/record/{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}'
    rospy.set_param("log", log)

    camera = Camera(serial_number=1)
    # cli = rospy.ServiceProxy("dobot_goal", dobot_control)
    # req = dobot_control._request_class()
    # req.goal = "reset"
    # reqr = cli(req)
           
    # 创建输入状态机
    sm = smach.StateMachine(outcomes=['input_success', 'input_failure'])
    # 输入部分
    with sm:
        smach.StateMachine.add('Moving Back', MoveToState(['initial location']), # 首先回到指令接收点
                                transitions={'success':'Speech',
                                             'failure':'Moving Back'})
        smach.StateMachine.add('Speech', SpeechState(), # 语音输入指令
                                transitions={'success':'Verify', 
                                             'failure':'Speech'})
        smach.StateMachine.add('Verify', VerifyState(), # 确认输入是否正确，正确则跳转下一状态，否则重新输入
                                transitions={'success':'Analyze', 
                                             'failure':'Speech'},
                                remapping={'verify_input': 'speech_output'})
        smach.StateMachine.add('Analyze', AnalyzeState(), # 大模型解析指令
                                transitions={'success':'input_success', 
                                             'failure':'input_failure'},
                                remapping={'analyze_input': 'verify_output'})
        
    # 创建并启动内部监测服务器
    sis = smach_ros.IntrospectionServer('gpsr_input_machine', sm, '/SM_ROOT')
    sis.start()
    pdf_num=1
    camera.wait_for_open_door()
    speech_to_text("door opened")
    try:
        while not rospy.is_shutdown():

            speech_state = 'input'
            # 启动状态机
            outcome = sm.execute()

            # 获得解析后的任务列表，用于执行状态机的创建
            operations = sm.userdata.analyze_output
            ori_command = sm.userdata.speech_output
            rospy.loginfo('operations:%s',operations)
            sis.stop()

            #operations=[['look_for_person','boy']]
            #operations=[['move_to','bedroom'],['look_for_person','Robin'],['follow','Robin']] # 测试用
            if len(operations) != 0:
                
                find_person_count = 0
                find_object_count = 0

                # 创建执行状态机
                sm_sub = create_state_machine(operations,ori_command)

                if sm_sub is None:
                    continue
                # 创建并启动内部监测服务器
                sis_sub = smach_ros.IntrospectionServer('gpsr_state_machine', sm_sub, '/SM_ROOT_SUB')
                sis_sub.start()

                # 启动执行状态机
                outcome_sub = sm_sub.execute()

                sis_sub.stop()

                pdf_file='/home/syw/code/robocup_nation/2024code/GPSR/src/record/command'+str(pdf_num)+'.pdf'
                pdf = FPDF()
                pdf.add_page()
                pdf.set_font('Arial', size=15)

                pdf_sentence = "Successfully execute the command"
                pdf.cell(200, 10, txt=pdf_sentence, ln=1, align='C')
                pdf_sentence =str(operations)
                pdf.cell(200, 10, txt=pdf_sentence, ln=2, align='C')
                pdf_sentence = time.asctime(time.localtime(time.time()))
                pdf.cell(200, 10, txt=pdf_sentence, ln=3, align='C')

                pdf.output(pdf_file)
                pdf_num+=1
            txt_to_pdf_fpdf(f"{log}.txt",f"{log}.pdf")
            log = f'/home/syw/code/robocup_nation/2024code/GPSR/src/record/{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}'
            rospy.set_param("log", log)
    except rospy.ROSInterruptException:
        pass