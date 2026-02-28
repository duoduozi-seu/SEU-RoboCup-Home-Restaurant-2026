#! /home/syw/anaconda3/envs/GPSR/bin/python
# -*- coding:UTF-8 -*-
import requests
import json
import re
import rospy

class Prompt:
    def __init__(self):
        self.task='analyze'
        self.command='Give me a cola'

# 服务名称
seuhome_llm_name = 'baidu_llm_service'
# 注意替换路径
plan_set_path = "/home/syw/code/robocup_nation/2024code/GPSR/src/seu_llm/scripts/plan_sets/plan_set_2025.json"

'''
    为了在程序中调用文心一言4.0大语言模型API，先在百度智能云创建API:KEY,替换下文'Authorization'即可
'''
Authorization = 'Bearer bce-v3/ALTAK-XtnGWJsMbwfdXWupUdgwo/b5b0b7853ac8bc6c80efc4e77bec4867c30281e5' 


def llm_service(req):
    # 根据服务请求，向大模型发送不同的prompt
    if req.task == 'analyze':
        return analyze(req)
    elif req.task == 'answer':
        return get_answer(req)

    print("Start to answer the question")
    tts_question = req.command
    url = "https://qianfan.baidubce.com/v2/chat/completions"
    # 创建当前prompt
    answer_prompt = "Please answer the following questions in English in no more than 30 words."
    answer_prompt += f"\n\n{tts_question}"
    payload = json.dumps({
         "model": "ernie-3.5-8k",
        "messages": [
            {
                "role": "user",
                "content": answer_prompt
            }
        ]
    })
    headers = {
        'Content-Type': 'application/json',
        'Authorization': Authorization 
    }
    pattern = r'"content":"(.*?)"'
    for i in range(5):
        response = requests.request("POST", url, headers=headers, data=payload)
        print("result is", response.text)
        match = re.search(pattern, response.text)
        if match:
            tts_answer = match.group(1)
            state = True
            errormsg = 'success'
            answer = tts_answer
            return state,errormsg,answer
        else: # 请求超过5次后，返回失败
            print("Failed to get answer. Trying again...")
            if i == 4:
                state = False
                errormsg = 'failure'
                answer = 'Sorry! I cannot answer your question.'
                return state,errormsg,answer
            else:
                payload = json.dumps({
                    "messages": [
                        {
                            "role": "user",
                            "content": "Please note the format of the answer!\n\n" + answer_prompt
                        }
                    ]
                })

# 回答问题
def get_answer(req):
    rospy.loginfo("Start to answer the question")
    tts_question = req.command
    url = "https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/completions_pro?access_token=" + get_access_token()
    # 创建当前prompt
    answer_prompt = "Please answer the following questions in English in no more than 30 words."
    answer_prompt += f"\n\n{tts_question}"
    payload = json.dumps({
        "messages": [
            {
                "role": "user",
                "content": answer_prompt
            }
        ]
    })
    headers = {
        'Content-Type': 'application/json'
    }
    pattern = r'"result":"(.*?)","is_truncated"'
    for i in range(5):
        response = requests.request("POST", url, headers=headers, data=payload)
        match = re.search(pattern, response.text)
        if match:
            tts_answer = match.group(1)
            state = True
            errormsg = 'success'
            answer = tts_answer
            return state,errormsg,answer
        else: # 请求超过5次后，返回失败
            rospy.loginfo("Failed to get answer. Trying again...")
            if i == 4:
                state = False
                errormsg = 'failure'
                answer = 'Sorry! I cannot answer your question.'
                return state,errormsg,answer
            else:
                payload = json.dumps({
                    "messages": [
                        {
                            "role": "user",
                            "content": "Please note the format of the answer!\n\n" + answer_prompt
                        }
                    ]
                })


# 解析指令
def analyze(req):
    print("Start to analyze the command")
    url = "https://qianfan.baidubce.com/v2/chat/completions"
    
    # 定义基元动作
    action_base = "move_to, speak, answer, look_for_person, look_for_obj, follow, lead_to, person_analyze, info_inform, grasp, deliver, put_on,answer"
    prompt = f"from actions import {action_base}"

    # 将action_base转换为列表形式
    action_base_lst = action_base.split(", ")

    # 添加场景obj、person
    # items = ['wine glass', 'kitchen cabinet', 'lime', 'mug', 'salmon', 'fruit', 'clothes', 'apple']
    # prompt += f"\n\nobjects = {items}"

    # 加载任务模板
    with open(plan_set_path, 'r') as f:
        tmp = json.load(f)
        prompt_egs = {}
        for k, v in tmp.items():
            prompt_egs[k] = v
        prompt += f"\n\nexamples = {prompt_egs}"

    # 加载任务提示
    prompt += "\n\nPlease answer the following questions with reference to the examples in the form of a nested python list without any modifiers. Your response is expected to be in the following format: 'Answer: [['actions', 'parameters'], ..., ['actions', 'parameters']]'."

    # 添加当前任务
    prompt_task = req.command
    print("Task: %s",prompt_task)

    prompt_known = f"There are something already known: \
        foods on the desk in the bedroom(kind contain chip, biscuit, bread) and  on the TV cabinet in the living room(kind contain biscuit, cookie, lays); \
        drinks on the side tble in the living room(kind contain sprite, cola, water) or on the storage table in the kitchen(kind contain sprite, cola, water), \
        cleaning stuffs on the dishwasher or sink in the kitchen(kind contain dishsoap, handwash, shampoo). \
        Please note that this information may not be entirely correct, such as food items in the bedroom that do not necessarily have chip\
        Others: dont need count without key how many;must analyze if want to tell somebody something about somebody; you must grasp obj before deliver;\
            you can direct move to furniture"
    # 创建当前prompt
    curr_prompt = f"{prompt}\n\n{prompt_known}\n\n{prompt_task}\n\t"
    payload = json.dumps({
        #"model": "ernie-3.5-8k",
        "model": "ernie-4.0-turbo-8k",
        "messages": [
            {
                "role": "user",
                "content": curr_prompt
            }
        ]
    })
    headers = {
        'Content-Type': 'application/json',
        'Authorization': Authorization 
    }
    
    pattern = r"\[\[.*?\]\]"
    prompt_add1 = "Please note the format of the answer!"
    prompt_add2 = "Please note that mandated subtasks need to be used to complete task planning"

    for i in range(5):
        response = requests.request("POST", url, headers=headers, data=payload)
        print("result is", response.text)
        # 将回答文本中的“result”存储在字符串中
        match = re.search(pattern, response.text)

        if match:
            action_str = match.group()
            print(action_str)
            # 将匹配到的字符串转换为函数列表
            action_lst = eval(match.group())
            for action in action_lst:
                if not action[0] in action_base_lst:
                    print(action[0])
            # 如果规划的子任务不在任务模板中，则提示用户重新回答
            if not all([action[0] in action_base_lst for action in action_lst]):
                print("Error: Planned subtasks are not executable. Trying again...")
                curr_prompt = f"{prompt}\n\n{prompt_task}\n\n{prompt_add2}"
                payload = json.dumps({
                    "messages": [
                        {
                            "role": "user",
                            "content": curr_prompt
                        }
                    ]
                })
                continue
            else:
                print("Task plan successfully!")
                state = True
                errormsg = 'success'
                answer = action_str
                break          
        else:
            print("Error: Failed to store result. Trying again...")
            # 添加提示词：请注意回答的格式!
            curr_prompt = f"{prompt}\n\n{prompt_task}\n\n{prompt_add1}"
            payload = json.dumps({
                "messages": [
                    {
                        "role": "user",
                        "content": curr_prompt
                    }
                ]
            })

        if i == 4:
            # 请求超过5次后，返回失败
            rospy.logerr("Error: Failed to store result after 5 attempts. Task planning failed.")
            action_str = ""
            state = False
            errormsg = 'failure'
            answer = action_str
            break

    return state,errormsg,answer

if __name__ == '__main__':
    # rospy.init_node("llm_service_node")
    # server = rospy.Service(seuhome_llm_name,seu_llm,llm_service)
    # rospy.loginfo("Ready to analyze the command")
    # rospy.spin()
    req =  Prompt()
    req.task = "analyze"
    req.command = "Who's the most handsome person in Canada"
    result = llm_service(req)
    print(result)