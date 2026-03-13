#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
"""GPSR 任务状态机主脚本。

该脚本整体流程：
1. 通过语音状态机获取并确认用户指令；
2. 通过 LLM 解析成动作序列；
3. 构建并执行动作状态机；
4. 记录日志并导出 PDF。
"""

from datetime import datetime
import os
import rospy
import smach
import smach_ros

from seu_smach_main.srv import *
from seu_look_for_object.msg import ObjectQueryResult
from seu_look_for_object.srv import (
    AnalyzeObject,
    AnalyzeObjectRequest,
    CountObject,
    CountObjectRequest,
    FindObject,
    FindObjectRequest,
    RelateObject,
    RelateObjectRequest,
)
import sys
import time
from fpdf import FPDF
from robot_msgs.msg import object_position
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from seu_voice.srv import seu_voice as seu_voice_srv, seu_voiceRequest

sys.path.append("/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src")
from common.camera import Camera

# ==============================
# 路径与导出配置（硬编码集中管理，方便统一修改）
# ==============================
RECORD_DIR = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src/record"  # str: 任务日志与 PDF 的根目录
LOG_TIME_FORMAT = "%Y-%m-%d %H:%M:%S"  # str: 日志文件名时间格式，例如 "2026-03-03 18:00:01"
COMMAND_PDF_PREFIX = "command"  # str: 命令执行结果 PDF 文件名前缀，示例 "command1.pdf"
COMMAND_PDF_TITLE = "Successfully execute the command"  # str: 命令执行结果 PDF 第一行固定文案

# ==============================
# ROS 服务名称常量
# ==============================
seuhome_speech_recognition_name = 'speech_recognition_service'  # str: 语音识别服务名
seuhome_llm_name = 'gemini_llm_service'  # str: LLM 服务名（任务解析与问答）
seuhome_find_person_name = 'find_person_service'  # str: 找人服务名
seuhome_find_object_name = 'find_object_service'  # str: 普通找物服务名
seuhome_relate_object_name = 'relate_object_service'  # str: 关系找物服务名
seuhome_count_object_name = 'count_object_service'  # str: 物体计数服务名
seuhome_analyze_object_name = 'analyze_object_service'  # str: 物体分析服务名
seuhome_nav_name = 'nav_service'  # str: 导航服务名
seuhome_follow_name = 'follow_service'  # str: 跟随服务名
seuhome_grasp_name = "dobot_server"  # str: 机械臂抓取服务名（当前脚本中未直接使用）
seuhome_voice_name = "voice_service"  # str: 语音合成服务名（基于 seu_voice）

# ==============================
# 全局运行态变量（状态机会在多个状态间共享）
# ==============================
find_person_count = 0  # int: 找人计数器，典型值为 0~N，当前主流程每轮会重置
find_object_count = 0  # int: 找物计数器，典型值为 0~N，当前主流程每轮会重置
speech_state = 'input'  # str: 语音状态，可能值 {'input', 'answer'}
ori_command = ""  # str: 原始自然语言指令文本
debug = True  # bool: 调试开关；True 时（除语音/LLM状态外）动作状态会直接返回 success
_voice_client = None  # rospy.ServiceProxy | None: voice_service 客户端单例

# 语音服务 busy 重试策略：每 1s 查询一次，最多查询 5 次（共等待 5s）。
VOICE_BUSY_ERROR_CODE = 1002
VOICE_BUSY_QUERY_INTERVAL_SEC = 1.0
VOICE_BUSY_MAX_QUERY_COUNT = 5
PKG_NAME_SMACH = "seu_smach_main"


def get_info(key):
    """读取 ROS 参数。

    Args:
        key (str): ROS 参数键名。

    Returns:
        Any | None: 参数存在则返回其值，不存在返回 None。
    """
    try:
        result = rospy.get_param(key)
    except:
        result = None
    return result


def build_empty_object_query_result(status_code="EMPTY", message=""):
    """构造一个空的物体查询结果。

    说明：
    1. 物体查询链路改造后，主状态机不再依赖 `obj_pos/obj_name/obj_count` 这类 ROS Param；
    2. 统一用 `ObjectQueryResult` 在状态机内部传递最近一次查询结果；
    3. 当查询尚未发生、查询失败、或服务异常时，都用这个空结果占位，避免后续状态读到脏数据。

    Args:
        status_code (str): 结果状态码，例如 `EMPTY`、`SERVICE_EXCEPTION`。
        message (str): 供日志和调试阅读的补充说明。

    Returns:
        ObjectQueryResult: 字段齐全但 `success=False` 的空结果对象。
    """
    result = ObjectQueryResult()
    result.success = False
    result.status_code = status_code
    result.message = message
    result.query_mode = ""
    result.target_class = ""
    result.reference_class = ""
    result.resolved_label = ""
    result.count = 0
    return result


def get_object_query_result(userdata):
    """从 SMACH userdata 中读取最近一次物体查询结果。"""
    try:
        return userdata.object_query_result
    except AttributeError:
        return None


def resolve_object_query_result_label(query_result, fallback=""):
    """从查询结果对象里提取“最终应使用的物体标签”。

    优先级：
    1. `resolved_label`：表示经过关系/排序/复核后最终选中的对象标签；
    2. `selected_object.label`：当服务尚未单独填充 resolved_label 时，回退到选中目标本身的标签；
    3. `fallback`：最后才使用动作参数中的原始文本。
    """
    if query_result is None:
        return fallback
    if getattr(query_result, "resolved_label", ""):
        return query_result.resolved_label
    selected_object = getattr(query_result, "selected_object", None)
    if selected_object is not None and getattr(selected_object, "label", ""):
        return selected_object.label
    return fallback


def resolve_query_object_label(userdata, fallback=""):
    """从最近一次物体查询结果里提取“最终应使用的物体标签”。
    """
    return resolve_object_query_result_label(
        get_object_query_result(userdata),
        fallback=fallback,
    )


def build_result_text(source_action, **fields):
    """将汇报事实整理为稳定的半结构化文本。"""
    lines = [f"source_action: {source_action}"]
    for key, value in fields.items():
        if value is None or value == "":
            continue
        if isinstance(value, bool):
            value = str(value).lower()
        lines.append(f"{key}: {value}")
    return "\n".join(lines)


def build_object_query_result_text(source_action, query_result, **extra_fields):
    """将物体查询结果整理为供 info_inform 使用的结果字符串。"""
    return build_result_text(
        source_action,
        success=getattr(query_result, "success", False),
        status_code=getattr(query_result, "status_code", ""),
        result_label=resolve_object_query_result_label(query_result),
        count=getattr(query_result, "count", 0) if source_action == "count_object" else None,
        **extra_fields,
    )


def build_person_count_result_text(feature):
    """从当前找人计数结果生成汇报文本。"""
    return build_result_text(
        "look_for_person",
        mode="count",
        success=True,
        person_feature=feature,
        count=get_info("person_count"),
    )


def build_person_analyze_result_text(feature):
    """从当前人物分析结果生成汇报文本。"""
    result_fields = {
        "success": True,
        "requested_feature": feature,
        "gesture": get_info("gesture"),
        "gender": get_info("gender"),
        "person_info": get_info("person_info"),
    }
    return build_result_text("person_analyze", **result_fields)


def rotate_robot(vel_pub, angular_speed, duration):
    """封装机器人原地旋转动作。

    Args:
        vel_pub (rospy.Publisher): `/cmd_vel` 发布器。
        angular_speed (float): 角速度（rad/s），正值逆时针，负值顺时针。
        duration (float): 旋转持续时间（秒）。
    """
    rate = rospy.Rate(10)
    end_time = rospy.Time.now().to_sec() + duration
    while rospy.Time.now().to_sec() < end_time:
        vel_cmd = Twist()
        vel_cmd.angular.z = angular_speed
        vel_pub.publish(vel_cmd)
        rate.sleep()
    # 停止旋转
    vel_pub.publish(Twist())


def build_log_base_path(now=None):
    """生成日志基路径（不带扩展名）。

    Args:
        now (datetime | None): 指定时间；None 时使用当前时间。

    Returns:
        str: 例如 `/.../record/2026-03-03 18:00:01`。
    """
    if now is None:
        now = datetime.now()
    return os.path.join(RECORD_DIR, now.strftime(LOG_TIME_FORMAT))


def txt_to_pdf_fpdf(input_txt, output_pdf):
    """将日志 TXT 文件逐行写入 PDF。

    Args:
        input_txt (str): 输入 txt 路径。
        output_pdf (str): 输出 pdf 路径。
    """
    try:
        # 若输入 txt 不存在，则直接返回，避免抛出文件不存在异常。
        if not os.path.exists(input_txt):
            return
        # 创建 PDF 对象并逐行写入文本。
        pdf = FPDF()
        pdf.add_page()
        pdf.set_font("Arial", size=12)
        with open(input_txt, "r", encoding="utf-8", errors="ignore") as file:
            for line in file:
                # FPDF (latin-1) 不支持中文或特殊 Unicode，这里做一次强制过滤转换
                # 确保不会因为编码问题导致整个机器人程序崩溃
                safe_line = line.encode("latin-1", "ignore").decode("latin-1")
                pdf.cell(0, 10, txt=safe_line.strip(), ln=True)

        pdf.output(output_pdf)
    except Exception as e:
        rospy.logerr(f"导出 PDF 报告失败（编码或文件错误），但不影响主流程: {e}")


def export_command_pdf(operations, pdf_num):
    """导出“单条命令执行结果”PDF，并返回更新后的序号。

    Args:
        operations (list): LLM 解析后的动作列表，典型结构为 `[['move_to', 'kitchen'], ...]`。
        pdf_num (int): 当前 PDF 序号，典型值为 1,2,3...

    Returns:
        int: 自增后的 PDF 序号（`pdf_num + 1`）。
    """
    try:
        # 确保输出目录存在，避免首次运行时目录不存在导致导出失败。
        if not os.path.exists(RECORD_DIR):
            os.makedirs(RECORD_DIR)

        pdf_file = os.path.join(RECORD_DIR, f"{COMMAND_PDF_PREFIX}{pdf_num}.pdf")
        pdf = FPDF()
        pdf.add_page()
        pdf.set_font('Arial', size=15)

        # 过滤非 latin-1 字符
        safe_ops = str(operations).encode("latin-1", "ignore").decode("latin-1")

        # PDF 第 1 行：固定标题；第 2 行：动作列表；第 3 行：导出时间。
        pdf.cell(200, 10, txt=COMMAND_PDF_TITLE, ln=1, align='C')
        pdf.cell(200, 10, txt=safe_ops, ln=2, align='C')
        pdf.cell(200, 10, txt=time.asctime(time.localtime(time.time())), ln=3, align='C')
        pdf.output(pdf_file)
    except Exception as e:
        rospy.logerr(f"导出命令 PDF 失败: {e}")
    
    return pdf_num + 1


# 定义输入部分状态类
# 语音输入
class SpeechState(smach.State):
    """语音输入状态。

    说明：
    - 当 `speech_state == 'input'` 时，采集任务指令；
    - 当 `speech_state == 'answer'` 时，采集问答问题。
    """

    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['success', 'failure'],
                            output_keys=['speech_output'])  # `speech_output: str`，识别得到的原始文本

    def execute(self, userdata):
        # 根据语音子任务模式选择识别类型。
        if speech_state == 'input':
            # 指令输入模式：默认调用语音识别服务；当服务返回 "input" 时回退到终端手输。
            if False:
                word = input("please enter command")
            else:
                word = srv_speech_recognition("command")  # str | -1
                if word == "input":
                    word = input("please enter command")
        elif speech_state == 'answer':
            # 问答输入模式：采集“问题”文本。
            word = srv_speech_recognition("answer")  # str | -1
       
        # 识别成功时写入 userdata，供 Verify/Analyze 等后续状态使用。
        if word != -1:
            userdata.speech_output = word
            rospy.loginfo('word: ' + word)
            return 'success'
        else:
            return 'failure'
        
# 确认输入是否正确
class VerifyState(smach.State):
    """语音确认状态。

    输入：`verify_input: str`（来自 SpeechState）。
    输出：`verify_output: str`（确认后的文本，供 Analyze/Answer 使用）。
    """

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success', 'failure'],
                             input_keys=['verify_input'],   # `verify_input: str`，上游识别文本
                             output_keys=['verify_output'])  # `verify_output: str`，确认通过后的文本

    def execute(self, userdata):
        verify_text = userdata.verify_input  # str: 待确认文本（指令或问题）
        userdata.verify_output = verify_text

        if speech_state == 'input':
            verify_str = "the command is: " + verify_text + ", is that right?"
        elif speech_state == 'answer':
            verify_str = "the question is: " + verify_text + ", is that right?"
        # 语音播报
        speech_to_text(verify_str)
        rospy.loginfo('%s', verify_str)
        result = self.verify()

        if result:
            userdata.verify_output = verify_text
            return 'success'
        else:
            return 'failure'

    def verify(self):
        """对“是否正确”进行语音确认。

        Returns:
            bool: True 表示用户确认，False 表示需重新输入。
        """
        result = srv_speech_recognition()  # str | -1
        if result == -1:  # 识别失败
            return False
        else:
            rospy.loginfo('word: %s', result)
            # 兼容多种“确认”关键词。
            if 'yes' in result or 'ok' in result:
                return True
            else:
                if 'right' in result:
                    return True
                else:
                    return False
                
# 大模型分析指令
class AnalyzeState(smach.State):
    """LLM 指令解析状态。

    输入：`analyze_input: str`（用户确认后的自然语言指令）。
    输出：`analyze_output: list`（动作序列，如 `[['move_to', 'kitchen'], ...]`）。
    """

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success', 'failure'],
                             input_keys=['analyze_input'],   # `analyze_input: str`，待解析指令
                             output_keys=['analyze_output'])  # `analyze_output: list`，动作列表

    def execute(self, userdata):
        # 该状态属于 LLM 链路，debug 模式下也不跳过。
        word = "Analyzing the command"
        speech_to_text(word)
        analyze_str = userdata.analyze_input  # str: 待解析命令文本
        rospy.loginfo("Task: %s", analyze_str)
        result = self.analyze(userdata)
        if result:
            return 'success'
        else:
            # 失败时置空动作列表，防止下游误执行历史数据。
            userdata.analyze_output = []
            return 'failure'

    def analyze(self, userdata):
        """调用 LLM 服务解析指令。"""
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
                action_lst = eval(response.answer)  # list: 解析后的动作序列（字符串转 Python 列表）
                rospy.loginfo('Analyze result: %s', response.answer)
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
    """导航状态：驱动机器人移动到目标位置。"""

    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0].replace('_', ' ')  # str: 目标地点名称，例如 "living room"

    def execute(self,userdata):
        # 调试模式下直接跳过导航执行，避免真实机器人移动。
        if debug:
            rospy.loginfo("debug=True，MoveToState 直接返回 success")
            return 'success'

        # 执行操作
        word = "Moving to "+ str(self.target)
        # 语音播报
        speech_to_text(word)
        # if self.target == 'initial location':  # 测试用
        #     return 'success'
        word="Successfully arrived at"+ str(self.target)
        result = self.move_to(self.target)
        if result:
            speech_to_text(word)
            return 'success'
        else:
            return 'failure'

    def move_to(self, target):
        """调用导航服务执行移动。"""
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
    """抓取状态：发布目标物体位置并控制机械臂执行抓取动作。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['object_query_result'],
        )
        # object_position 发布器：向机械臂或感知链路同步目标物体位姿。
        self.detect_result_pub = rospy.Publisher(
            '/detect_result2', object_position, queue_size=10
        )
        self.cli = rospy.ServiceProxy("dobot_goal", dobot_control)  # ServiceProxy: 机械臂控制服务客户端
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher: 底盘速度控制发布器
        self.msg = String()  # String: 占位消息对象，当前实现未使用
        self.obj = arg[0] if arg else ""  # str: 规划器给出的对象名；真正执行时优先取最近一次查询结果

    def execute(self, userdata):
        # 调试模式下直接跳过抓取执行，避免机械臂和底盘动作。
        if debug:
            rospy.loginfo("debug=True，GraspState 直接返回 success")
            return 'success'

        query_result = get_object_query_result(userdata)
        if query_result is None or not getattr(query_result, "success", False):
            rospy.logerr("GraspState 缺少有效 object_query_result，无法执行抓取")
            return 'failure'

        selected_object = query_result.selected_object
        if not getattr(selected_object, "pose_valid", False):
            rospy.logerr("GraspState 收到的 selected_object 不包含有效三维位姿")
            return 'failure'

        # 将统一查询结果转换成旧机械臂链路仍在使用的 object_position 消息。
        p = object_position()
        p.label = resolve_query_object_label(userdata, self.obj)
        p.x = selected_object.pose.pose.position.x
        p.y = selected_object.pose.pose.position.y
        p.z = selected_object.pose.pose.position.z
        self.detect_result_pub.publish(p)

        rospy.loginfo("机械臂正在向前抓取...")
        speech_to_text(f"start to grasp {p.label}")
        self.req = dobot_control._request_class()
        self.req.goal = "groceries_catch1"
        self.cli(self.req)

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
        self.cli(self.req)
        speech_to_text("finish grasp")
        rospy.loginfo("机械臂抓取成功")
        return 'success'



# 传递
class DeliverState(smach.State):
    """递交状态：机械臂把物体递给目标人物，并等待口头确认。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['object_query_result'],
        )
        self.target = arg[0] if arg else ""  # str: 规划器里的对象占位；递交动作会优先播报最近一次查询到的对象名
        self.cli = rospy.ServiceProxy("dobot_goal", dobot_control)  # ServiceProxy: 机械臂控制服务
        self.msg = String()  # String: 占位消息对象，当前实现未使用
        self.word = ""  # str: 实时语音识别最近一次结果
        rospy.Subscriber('/real_time_speech_recognition_topic', String, self.speech_recognition_callback)

    def execute(self, userdata):
        # 调试模式下直接跳过递交执行，避免机械臂动作和实时语音等待。
        if debug:
            rospy.loginfo("debug=True，DeliverState 直接返回 success")
            return 'success'

        # 语音播报
        target_name = resolve_query_object_label(userdata, self.target)
        speech_to_text(f"Passing {target_name if target_name else 'the object'}")
        self.req = dobot_control._request_class()
        self.req.goal = "gpsr_give1"
        reqr = self.cli(self.req)

        speech_to_text("please say ok after you get it")
        rospy.set_param("real_time_speech_recognition_enable", True)
        last_time = time.time()  # float: 最近一次提示时间戳
        times = 0  # int: 二次提示计数，超过阈值后自动结束等待
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
    """放置状态：控制机械臂将物体放到目标位置。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['object_query_result'],
        )
        self.obj = arg[0]  # str: 物体名称；若值为 `current_query_object`，则运行时从最近一次查询结果解析
        self.target = arg[1]  # str: 放置位置名称
        self.cli = rospy.ServiceProxy("dobot_goal", dobot_control)  # ServiceProxy: 机械臂控制服务
        self.msg = String()  # String: 占位消息对象，当前实现未使用
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher: 底盘速度控制发布器

    def execute(self, userdata):
        # 调试模式下直接跳过放置执行，避免机械臂和底盘动作。
        if debug:
            rospy.loginfo("debug=True，PutOnState 直接返回 success")
            return 'success'

        # 语音播报
        obj_name = resolve_query_object_label(userdata, self.obj)
        speech_to_text(f"Put {obj_name if obj_name else self.obj} to {self.target}")
        result = self.pass_to(self.target)
        if result:
            return 'success'
        else:
            return 'failure'

    def pass_to(self, target): # 实际功能尚未完全编写
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
    """找人状态：调用视觉/感知服务按特征查找目标人物。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['result_text'],
        )
        self.target = arg[0]  # str: 找人特征（可能是人名、衣着、属性等）
        self.count = arg[1]  # str | int: 查找次序/人数参数，具体语义由 find_person 服务定义

    def execute(self,userdata):
        # 调试模式下直接跳过找人流程，避免触发感知服务。
        if debug:
            rospy.loginfo("debug=True，LookForPersonState 直接返回 success")
            return 'success'

        # 执行操作
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
            if self.count == "count":
                userdata.result_text = build_person_count_result_text(self.target)
            speech_to_text(word)
            return 'success'
        else:
            return 'failure'

    def look_for_person(self, target):
        """调用找人服务。"""
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
    """人物分析状态：调用人物属性分析服务。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['result_text'],
        )
        self.feature = arg[0]  # str: 需要分析的人物特征关键字

    def execute(self, userdata):
        # 调试模式下直接跳过人物分析流程，避免触发分析服务。
        if debug:
            rospy.loginfo("debug=True，PersonAnalyzeState 直接返回 success")
            return 'success'

        # 执行操作
        word = "Start analyse people"
        # 语音播报
        speech_to_text(word,speak=False)

        result = self.person_analyse(self.feature)
        word = "Successfully analyse person" + str(self.feature)
        if result:
            userdata.result_text = build_person_analyze_result_text(self.feature)
            speech_to_text(word,speak=False)
            return 'success'
        else:
            return 'failure'

    def person_analyse(self,feature):
        """调用人物分析服务。"""
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

# 物体分析
class AnalyzeObjectState(smach.State):
    """物体分析状态：调用 analyze_object_service 返回排序/比较后的目标结果。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['object_query_result', 'result_text'],
        )
        self.target_class = arg[0] if len(arg) > 0 else ""  # str: 目标类别，例如 cleaning_supply / snack / any
        self.metric = arg[1] if len(arg) > 1 else ""  # str: 分析指标，例如 biggest / heaviest
        self.search_scope = arg[2] if len(arg) > 2 else ""  # str: 搜索范围，由 object 查询模块内部负责巡航
        self.timeout_sec = float(arg[3]) if len(arg) > 3 else 180.0  # float: 查询超时时间，默认 180 秒

    def execute(self, userdata):
        if debug:
            rospy.loginfo("debug=True，AnalyzeObjectState 直接返回 success")
            return 'success'

        speech_to_text(
            f"Analyzing {self.target_class} in {self.search_scope} by {self.metric}"
        )
        result = self.call_service()
        userdata.object_query_result = result
        if result.success:
            userdata.result_text = build_object_query_result_text(
                "analyze_object",
                result,
                target_class=self.target_class,
                metric=self.metric,
                search_scope=self.search_scope,
            )
            speech_to_text(f"Successfully analyzed {result.resolved_label}")
            return 'success'
        return 'failure'

    def call_service(self):
        """调用 analyze_object_service。"""
        srv_name = seuhome_analyze_object_name
        client = rospy.ServiceProxy(srv_name, AnalyzeObject)
        rospy.wait_for_service(srv_name)
        rospy.loginfo("%s available", srv_name)
        try:
            request = AnalyzeObjectRequest()
            request.target_class = self.target_class
            request.metric = self.metric
            request.search_scope = self.search_scope
            request.timeout_sec = self.timeout_sec
            response = client.call(request)
            return response.result
        except rospy.ServiceException as e:
            rospy.logerr("Service call Failed: %s: %s", srv_name, e)
            return build_empty_object_query_result(
                status_code="SERVICE_EXCEPTION",
                message=str(e),
            )


class CountObjectState(smach.State):
    """物体计数状态：调用 count_object_service 返回给定范围内的目标数量。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['object_query_result', 'result_text'],
        )
        self.target_class = arg[0] if len(arg) > 0 else ""  # str: 目标类别
        self.search_scope = arg[1] if len(arg) > 1 else ""  # str: 搜索范围
        self.timeout_sec = float(arg[2]) if len(arg) > 2 else 180.0  # float: 查询超时时间

    def execute(self, userdata):
        if debug:
            rospy.loginfo("debug=True，CountObjectState 直接返回 success")
            return 'success'

        speech_to_text(f"Counting {self.target_class} in {self.search_scope}")
        result = self.call_service()
        userdata.object_query_result = result
        if result.success:
            userdata.result_text = build_object_query_result_text(
                "count_object",
                result,
                target_class=self.target_class,
                search_scope=self.search_scope,
            )
            speech_to_text(f"Count finished, total {result.count}")
            return 'success'
        return 'failure'

    def call_service(self):
        """调用 count_object_service。"""
        srv_name = seuhome_count_object_name
        client = rospy.ServiceProxy(srv_name, CountObject)
        rospy.wait_for_service(srv_name)
        rospy.loginfo("%s available", srv_name)
        try:
            request = CountObjectRequest()
            request.target_class = self.target_class
            request.search_scope = self.search_scope
            request.timeout_sec = self.timeout_sec
            response = client.call(request)
            return response.result
        except rospy.ServiceException as e:
            rospy.logerr("Service call Failed: %s: %s", srv_name, e)
            return build_empty_object_query_result(
                status_code="SERVICE_EXCEPTION",
                message=str(e),
            )


class FindObjectState(smach.State):
    """普通找物状态：调用 find_object_service 在给定范围内搜索目标物体。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['object_query_result'],
        )
        self.target_class = arg[0] if len(arg) > 0 else ""  # str: 目标类别
        self.search_scope = arg[1] if len(arg) > 1 else ""  # str: 搜索范围
        self.timeout_sec = float(arg[2]) if len(arg) > 2 else 180.0  # float: 查询超时时间

    def execute(self, userdata):
        # if debug:
        #     rospy.loginfo("debug=True，FindObjectState 直接返回 success")
        #     return 'success'

        speech_to_text(f"Looking for {self.target_class} in {self.search_scope}")
        result = self.call_service()
        userdata.object_query_result = result
        if result.success:
            speech_to_text(f"Successfully found {resolve_query_object_label(userdata, self.target_class)}")
            return 'success'
        return 'failure'

    def call_service(self):
        """调用 find_object_service。"""
        srv_name = seuhome_find_object_name
        client = rospy.ServiceProxy(srv_name, FindObject)
        rospy.wait_for_service(srv_name)
        rospy.loginfo("%s available", srv_name)
        try:
            request = FindObjectRequest()
            request.target_class = self.target_class
            request.search_scope = self.search_scope
            request.timeout_sec = self.timeout_sec
            response = client.call(request)
            return response.result
        except rospy.ServiceException as e:
            rospy.logerr("Service call Failed: %s: %s", srv_name, e)
            return build_empty_object_query_result(
                status_code="SERVICE_EXCEPTION",
                message=str(e),
            )


class RelateObjectState(smach.State):
    """关系找物状态：调用 relate_object_service 查询带关系约束的目标物体。"""

    def __init__(self, arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['object_query_result', 'result_text'],
        )
        self.target_class = arg[0] if len(arg) > 0 else ""  # str: 目标类别，可为 any
        self.reference_class = arg[1] if len(arg) > 1 else ""  # str: 参考物类别
        self.relation = arg[2] if len(arg) > 2 else ""  # str: 关系词，例如 behind / top / left_of
        self.search_scope = arg[3] if len(arg) > 3 else ""  # str: 搜索范围
        self.timeout_sec = float(arg[4]) if len(arg) > 4 else 180.0  # float: 查询超时时间

    def execute(self, userdata):
        if debug:
            rospy.loginfo("debug=True，RelateObjectState 直接返回 success")
            return 'success'

        speech_to_text(
            f"Looking for {self.target_class} {self.relation} {self.reference_class} in {self.search_scope}"
        )
        result = self.call_service()
        userdata.object_query_result = result
        if result.success:
            userdata.result_text = build_object_query_result_text(
                "relate_object",
                result,
                target_class=self.target_class,
                reference_class=self.reference_class,
                relation=self.relation,
                search_scope=self.search_scope,
            )
            speech_to_text(
                f"Successfully found {resolve_query_object_label(userdata, self.target_class)}"
            )
            return 'success'
        return 'failure'

    def call_service(self):
        """调用 relate_object_service。"""
        srv_name = seuhome_relate_object_name
        client = rospy.ServiceProxy(srv_name, RelateObject)
        rospy.wait_for_service(srv_name)
        rospy.loginfo("%s available", srv_name)
        try:
            request = RelateObjectRequest()
            request.target_class = self.target_class
            request.reference_class = self.reference_class
            request.relation = self.relation
            request.search_scope = self.search_scope
            request.timeout_sec = self.timeout_sec
            response = client.call(request)
            return response.result
        except rospy.ServiceException as e:
            rospy.logerr("Service call Failed: %s: %s", srv_name, e)
            return build_empty_object_query_result(
                status_code="SERVICE_EXCEPTION",
                message=str(e),
            )

# 跟随 
class FollowState(smach.State):
    """跟随状态：在指定地点跟随目标人物。"""

    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.location = arg[0]  # str: 跟随任务触发地点或目标区域
        self.target = "person"  # str: 跟随对象占位值，当前流程中固定为 "person"

    def execute(self,userdata):
        # 调试模式下直接跳过跟随流程，避免触发跟随服务。
        if debug:
            rospy.loginfo("debug=True，FollowState 直接返回 success")
            return 'success'

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

    def follow(self, target):
        """调用跟随服务。"""
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
    """引导状态：带领用户到指定目标位置。"""

    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0]  # str: 引导目标地点

    def execute(self, userdata):
        # 调试模式下直接跳过引导流程，避免触发引导服务。
        if debug:
            rospy.loginfo("debug=True，LeadToState 直接返回 success")
            return 'success'

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
    """语音播报状态：将指定文本直接播报。"""

    def __init__(self, arg):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target = arg[0]  # str: 需要播报的文本内容

    def execute(self,userdata):
        # 调试模式下直接跳过播报动作，避免真实语音输出。
        if debug:
            rospy.loginfo("debug=True，SpeakState 直接返回 success")
            return 'success'

        # 执行操作
        result = self.speak(self.target)
        if result:
            return 'success'
        else:
            return 'failure'

    def speak(self, target):
        """调用语音播报；内容通常来自 LLM 解析阶段。"""
        speech_to_text(target)
        return True

# 回答问题    
class AnswerState(smach.State):
    """问答状态：调用 LLM 生成答案并播报。"""

    def __init__(self, arg=None):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['answer_input'],
        )
        self.question = arg[0] if arg else ""  # str: 固定问题；为空时从 userdata.answer_input 动态读取

    def execute(self,userdata):
        # 执行操作
        question = self.question if self.question else userdata.answer_input
        rospy.loginfo("The question is: %s",question)
        result = self.answer(question)
        if result:
            return 'success'
        else:
            return 'failure'

    def answer(self,question):
        """调用 LLM 问答服务。"""
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
    """信息告知状态：根据任务类型返回结构化信息。"""

    def __init__(self,arg):
        smach.State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['ori_command', 'result_text'],
            output_keys=['info_kind'],
        )
        self.info_kind = arg[0]  # str: 需要告知的信息类型（由上游动作参数提供）

    def execute(self,userdata):
        # 调试模式下直接跳过信息告知流程，避免触发 info_inform_service。
        if debug:
            rospy.loginfo("debug=True，InfoInformState 直接返回 success")
            return 'success'

        userdata.info_kind = self.info_kind
        question = userdata.ori_command
        result_text = userdata.result_text
        rospy.loginfo("The question is: %s", question)
        rospy.loginfo("InfoInformState result_text: %s", result_text)
        result = self.answer(question, userdata.info_kind, result_text)
        if result:
            return 'success'
        else:
            return 'failure'

    def answer(self, question, info_kind, result_text):
        """调用信息告知服务。"""
        srv_name = "info_inform_service"  # 当前服务的名称
        client = rospy.ServiceProxy(srv_name, seu_info_inform)  # 当前操作的服务
        rospy.wait_for_service(srv_name)  # 等待服务
        rospy.loginfo(srv_name + ' available')
        try:
            request = seu_info_informRequest()
            request.info_kind = info_kind
            request.ori_command = question
            request.result_text = result_text
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

def _print_voice_busy_skip(pkg_name):
    """按需求输出红色 busy 提示，提示后续逻辑继续执行。"""
    msg = f"[{pkg_name}]：voice service busy，skip speech。"
    sys.stderr.write(f"\033[31m{msg}\033[0m\n")
    sys.stderr.flush()


def _get_voice_client():
    """获取 voice_service 的单例客户端，减少重复创建开销。"""
    global _voice_client
    if _voice_client is None:
        _voice_client = rospy.ServiceProxy(seuhome_voice_name, seu_voice_srv)
    return _voice_client


def _call_voice_service_blocking(text):
    """调用 voice_service 进行阻塞播报，并处理 busy 轮询逻辑。

    规则：
    1. 正常返回 `state=True` 即表示“播报已结束”；
    2. 若返回 busy（errorcode=1002），则每 1s 重试一次，最多 5 次；
    3. 若 5 次后仍 busy，打印红色提示并跳过语音；
    4. 其他错误直接记录日志并返回失败。
    """
    if text is None:
        return False
    text = str(text).strip()
    if not text:
        return False

    client = _get_voice_client()
    req = seu_voiceRequest(text=text)

    try:
        # 为避免节点永久卡死，等待服务时设置超时；超时后跳过语音。
        rospy.wait_for_service(seuhome_voice_name, timeout=1.0)
    except rospy.ROSException as e:
        rospy.logerr("voice service unavailable: %s", e)
        return False

    def _call_once():
        try:
            return client.call(req)
        except rospy.ServiceException as call_error:
            rospy.logerr("voice service call failed: %s", call_error)
            return None

    # 第一次调用：正常情况下会阻塞直到播报结束。
    response = _call_once()
    if response is None:
        return False
    if response.state:
        return True

    # 非 busy 错误：直接失败。
    if response.errorcode != VOICE_BUSY_ERROR_CODE:
        rospy.logerr("voice service failed: errorcode=%s, errormsg=%s",
                     response.errorcode, response.errormsg)
        return False

    # busy：按要求每 1s 查询一次，最多 5 次。
    for _ in range(VOICE_BUSY_MAX_QUERY_COUNT):
        rospy.sleep(VOICE_BUSY_QUERY_INTERVAL_SEC)
        response = _call_once()
        if response is None:
            return False
        if response.state:
            return True
        if response.errorcode != VOICE_BUSY_ERROR_CODE:
            rospy.logerr("voice service failed after busy retry: errorcode=%s, errormsg=%s",
                         response.errorcode, response.errormsg)
            return False

    _print_voice_busy_skip(PKG_NAME_SMACH)
    return False


def speech_to_text(data, log_file= None,speak = True):
    """语音播报并记录日志。

    Args:
        data (str): 需要播报和写日志的文本内容。
        log_file (str | None): 日志基路径（不带扩展名）；None 时从 ROS 参数 `log` 读取。
        speak (bool): 是否实际播报；False 时仅写日志。
    """
    try:
        if speak:
            # 通过 voice_service 阻塞播报；若失败（含 busy 超时）则按策略跳过。
            _call_voice_service_blocking(data)
        if log_file is None:
            log_file = get_info('log')  # str | None: 例如 `/.../record/2026-03-03 18:00:01`
        if not log_file:
            return
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
    """调用语音识别服务。

    Args:
        task (str): 识别任务类型，常见值：
            - "command": 指令识别
            - "answer": 问题识别
            - "none": 默认值

    Returns:
        str | int: 成功返回识别文本；失败返回 -1。
    """
    srv_name = seuhome_speech_recognition_name  # 当前服务的名称
    client = rospy.ServiceProxy(srv_name, seu_speech_recognition)  # 当前操作的服务
    rospy.wait_for_service(srv_name)  # 等待服务
    rospy.loginfo(srv_name + ' available')
    try:
        request = seu_speech_recognitionRequest()
        request.task = task           #speech_state==input->task==command; speech_state==answer->task==answer
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
    """根据 LLM 解析结果动态创建动作状态机。

    Args:
        actions (list[list[str]]): 动作序列，每个元素为 `[action_name, arg1, arg2, ...]`。
        ori_command (str): 原始自然语言指令文本（用于后续信息类状态参考）。

    Returns:
        smach.StateMachine | None: 构建成功返回状态机；若参数不合法（如 lead_to 缺失目标）返回 None。
    """
    # 将“动作名 + 参数 + 序号”拼接成唯一状态名，避免同名状态冲突。
    state_names = [f'{action[0]} {action[1:]} {index}' for index, action in enumerate(actions)]
    state_names.append('sub_success') # 在末端添加“sub_success”，作为最后一个状态的成功跳转目标

    # 创建子状态机并挂载原始指令到 userdata，供部分状态读取。
    sm = smach.StateMachine(outcomes=['sub_success', 'sub_failure'])
    sm.userdata.ori_command = ori_command
    sm.userdata.info_kind = ""
    sm.userdata.result_text = ""
    sm.userdata.answer_input = ""
    sm.userdata.object_query_result = build_empty_object_query_result(
        status_code="INIT",
        message="object query result not set yet",
    )
    # 根据解析出的动作类型，逐项注册对应状态节点。
    with sm:
        for i in range(len(actions)):
            action = actions[i][0]  # str: 动作名，可能值如 move_to/grasp/deliver/put_on/look_for_person/follow...
            arg = actions[i][1:]  # list[str]: 动作参数列表，长度和语义由动作类型决定
            if action == 'move_to':
                smach.StateMachine.add(state_names[i],
                                       MoveToState(arg),
                                       transitions={'success': state_names[i+1],# 当前状态成功，跳转下一状态，否则重复当前状态。下同
                                                    'failure': state_names[i]})
            elif action == 'grasp':
                smach.StateMachine.add(state_names[i],
                                       GraspState(arg),
                                       transitions={'success': state_names[i+1],# grasp和pass_to具体逻辑代码未编写
                                                    'failure': state_names[i]},
                                       remapping={'object_query_result': 'object_query_result'})
            elif action == 'deliver':
                smach.StateMachine.add(state_names[i],
                                       DeliverState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'object_query_result': 'object_query_result'})
            elif action == 'put_on':
                smach.StateMachine.add(state_names[i],
                                       PutOnState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'object_query_result': 'object_query_result'})
            elif action == 'analyze_object':
                smach.StateMachine.add(state_names[i],
                                       AnalyzeObjectState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'object_query_result': 'object_query_result',
                                                  'result_text': 'result_text'})
            elif action == 'count_object':
                smach.StateMachine.add(state_names[i],
                                       CountObjectState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'object_query_result': 'object_query_result',
                                                  'result_text': 'result_text'})
            elif action == 'person_analyze':
                smach.StateMachine.add(state_names[i],
                                       PersonAnalyzeState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'result_text': 'result_text'})
            elif action == 'info_inform':
                smach.StateMachine.add(state_names[i],
                                       InfoInformState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'ori_command': 'ori_command',
                                                  'info_kind': 'info_kind',
                                                  'result_text': 'result_text'})
            elif action == 'look_for_person':
                smach.StateMachine.add(state_names[i],
                                       LookForPersonState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'result_text': 'result_text'})
            elif action == 'find_object':
                smach.StateMachine.add(state_names[i],
                                       FindObjectState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'object_query_result': 'object_query_result'})
            elif action == 'relate_object':
                smach.StateMachine.add(state_names[i],
                                       RelateObjectState(arg),
                                       transitions={'success': state_names[i+1],
                                                    'failure': state_names[i]},
                                       remapping={'object_query_result': 'object_query_result',
                                                  'result_text': 'result_text'})
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
                                                    'failure': 'sub_failure'},
                                       remapping={'answer_input': 'answer_input'})
            else:
                rospy.logerr("Unsupported action in create_state_machine: %s", action)
                return None

    return sm

# 创建回答问题状态机
def create_answer_smach():
    """创建问答子状态机（语音输入问题 -> 确认 -> LLM 回答）。"""
    global speech_state
    speech_state = 'answer'  # str: 切换到问答语音模式，SpeechState/VerifyState 会据此调整提示语
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
    find_person_count = 0
    find_object_count = 0
    log = build_log_base_path()  # str: 日志基路径（不带扩展名），实际会写出 `{log}.txt` 与 `{log}.pdf`
    rospy.set_param("log", log)

    rospy.loginfo("\033[32m[smach]\033[0m initialized")  # 输出彩色[smach]标识，提示状态机节点已启动

    if not debug:
      camera = Camera(serial_number=1)

           
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
    pdf_num = 1  # int: 命令执行摘要 PDF 的序号计数器，输出为 command1.pdf、command2.pdf...
    if not debug:
      camera.wait_for_open_door()
    speech_to_text("door opened")
    try:
        while not rospy.is_shutdown():

            speech_state = 'input'
            # 启动状态机
            outcome = sm.execute()

            # 获得解析后的任务列表，用于执行状态机的创建
            operations = sm.userdata.analyze_output  # list[list[str]]: 解析后的动作序列
            ori_command = sm.userdata.speech_output  # str: 当前轮原始语音指令
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

                # 将命令执行结果导出为独立 PDF（例如 command1.pdf）。
                pdf_num = export_command_pdf(operations, pdf_num)
            # 将本轮日志 txt 同步转为同名 pdf，便于赛后检索与汇报。
            txt_to_pdf_fpdf(f"{log}.txt", f"{log}.pdf")
            # 每轮循环更新日志基路径，避免不同命令日志混写到同一文件。
            log = build_log_base_path()
            rospy.set_param("log", log)
    except rospy.ROSInterruptException:
        pass
