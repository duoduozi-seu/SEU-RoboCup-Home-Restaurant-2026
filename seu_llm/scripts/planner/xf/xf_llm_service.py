#!/usr/bin/env python3
# -*- coding:UTF-8 -*-

# 该代码用于调用讯飞星火大模型API进行任务规划及问题回答(已和smach整合)

# 讯飞星火大模型Api具体的申请及使用流程可参考链接 https://github.com/zibuyu2015831/xfyun-spark-api
# 讯飞星火大模型有免费二百万token（讯飞的token计算,1tokens 约等于1.5个中文汉字 或者 0.8个英文单词，所以总量相当于160万字的英文文本交互，600万字的中文文本交互）
# 若目前的免费额度用完，可参考上述链接进行新用户的申请

# 使用讯飞星火大模型时，需要将spark_speaker.py中的config_path参数修改为配置文件(config.json)的绝对路径，
# xf_llm_service.py中的plan_set_root参数修改为喂给大模型的例子(plan_set8.json)的绝对路径

import json
import re
import rospy
from seu_llm.srv import *
from spark_speaker import SparkSpeaker

seuhome_llm_name = 'xf_llm_service'

# plan_set_root 为喂给大模型的例子(plan_set.json)的绝对路径，需要自行修改
plan_set_root = '/home/syw/robocup/GPSR_code/seu_llm/scripts/plan_sets/plan_set.json'


# 判断当前是调用大模型进行任务规划还是问题回答
def llm_service(req):
    # 如果是进行任务规划则跳转到函数analyze()
    if req.task == 'analyze':
        return analyze(req)
    # 如果是进行问题回答则跳转到函数get_answer(req)
    elif req.task == 'answer':
        return get_answer(req)


# 调用大模型进行问题回答
def get_answer(req):
    rospy.loginfo("Start to answer the question")
    tts_question = req.command #当前问题

    # 创建当前prompt
    answer_prompt = "Please answer the following questions in English in no more than 30 words."
    # 给大模型prompt
    speaker = SparkSpeaker(answer_prompt)

    answer_prompt += f"\n\n{tts_question}"
    # 询问大模型问题，response为大模型返回的答复
    response = speaker.ask(answer_prompt)

    # 将response中'Answer:'后的内容转换为文本tts_answer
    tts_answer = response.split('Answer:')[1]
    # print(tts_answer)
    state = True
    errormsg = 'success'
    answer = tts_answer

    return seu_llmResponse(state, errormsg, answer)


def analyze(req):
    rospy.loginfo("Start to analyze the command")

    # 定义基元动作
    action_base = "move_to, find_object, relate_object, count_object, analyze_object, look_for_person, follow, grasp, pass_to, speak, answer"
    prompt = f"from actions import {action_base}"

    # 将action_base转换为列表形式
    action_base_lst = action_base.split(", ")

    # 加载任务模板
    with open(plan_set_root, 'r') as f:
        tmp = json.load(f)
        prompt_egs = {}
        for k, v in tmp.items():
            prompt_egs[k] = v
        prompt += f"\n\nexamples = {prompt_egs}"

    # 加载任务提示
    prompt += "\n\nAnswer the following question with reference to the examples in the form of a nested python list without any modifiers. Your response is expected to be in the following format: 'Answer: [['actions', 'parameters'], ..., ['actions', 'parameters']]'."
    speaker = SparkSpeaker(prompt)

    # 添加当前任务
    prompt_task = req.command
    rospy.loginfo("Task: %s", prompt_task)

    # 创建当前prompt
    curr_prompt = f"{prompt}\n\n{prompt_task}\n\t"

    pattern = r"\[\[.*?\]\]"
    prompt_add1 = "Please note the format of the answer!"
    prompt_add2 = "Please note that mandated subtasks need to be used to complete task planning"

    for i in range(5):
        # 调用大模型进行任务规划
        response = speaker.ask(curr_prompt)

        # 进行正确格式的回答文本存储
        match = re.search(pattern, response)
        # 回答文本格式正确，进行下一步分析
        if match:
            action_str = match.group()
            rospy.loginfo(action_str)
            # 将匹配到的字符串转换为函数列表
            action_lst = eval(match.group())
            # 如果规划的子任务不在任务模板中，则提示用户重新回答
            if not all([action in action_base_lst for action, _ in action_lst]):
                rospy.logerr("Error: Planned subtasks are not executable. Trying again...")
                curr_prompt = f"{prompt}\n\n{prompt_task}\n\n{prompt_add2}"
            else:
                rospy.loginfo("Task plan successfully!")
                state = True
                errormsg = 'success'
                answer = action_str
                break
        # 回答文本格式错误，重新分析
        else:
            rospy.logerr("Error: Failed to store result. Trying again...")
            # 添加提示词：请注意回答的格式!
            curr_prompt = f"{prompt}\n\n{prompt_task}\n\n{prompt_add1}"

        # 请求超过5次后，返回失败
        if i == 4:
            rospy.logerr("Error: Failed to store result after 5 attempts. Task planning failed.")
            action_str = ""
            state = False
            errormsg = 'failure'
            answer = action_str
            break

    return seu_llmResponse(state, errormsg, answer)


if __name__ == '__main__':
    rospy.init_node("llm_service_node")
    server = rospy.Service(seuhome_llm_name, seu_llm, llm_service)
    rospy.loginfo("Ready to analyze the command")
    rospy.spin()
