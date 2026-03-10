#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
import requests
import json
import re
import rospy
from seu_llm.srv import *

# 服务名称
seuhome_llm_name = 'baidu_llm_service'
# 注意替换路径
plan_set_path = "/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_llm/scripts/plan_sets/plan_set_2025.json"

'''
    为了在程序中调用文心一言4.0大语言模型API，先在百度智能云创建API:KEY,替换下文'Authorization'即可
'''
Authorization = 'Bearer bce-v3/ALTAK-XtnGWJsMbwfdXWupUdgwo/b5b0b7853ac8bc6c80efc4e77bec4867c30281e5' 

answer_reference = "question:Who's the most handsome person in Canada?,answer:I that Justin Trudeau is very handsome.;question:How many time zones are there in Canada?,answer:Canada spans almost 10 million square km and comprises 6 time zones;question:What's the longest street in the world?,answer:Yonge Street in Ontario is the longest street in the world.;question:How long is Yonge Street in Ontario?,answer:Yonge street is almost 2,000 km, starting at Lake Ontario, and running north to the Minnesota border.;question:What's the name of the bear cub exported from Canada to the London Zoo in 1915?,answer:The bear cub was named Winnipeg. It inspired the stories of Winnie-the-Pooh.;question:Where was the Blackberry Smartphone developed?,answer:It was developed in Ontario, at Research In Motion's Waterloo offices.;question:What is the world's largest coin?,answer:The Big Nickel in Sudbury, Ontario. It is nine meters in diameter.;question:In what year was Canada invaded by the USA for the first time?,answer:The first time that the USA invaded Canada was in 1775;question:What year was Canada invaded by the USA for the second time?,answer:The USA invaded Canada a second time in 1812.;question:What country holds the record for the most gold medals at the Winter Olympics?,answer:Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics.;question:Who coined the term Beatlemania?,answer:Sandy Gardiner, a journalist of the Ottawa Journal.;question:Why is Canada named Canada?,answer:French explorers misunderstood the local native word Kanata, which means village.;question:When was The Mounted Police formed?,answer:The Mounted Police was formed in 1873.;question:When was The Royal Canadian Mounted Police formed?,answer:In 1920, when The Mounted Police merged with the Dominion Police.;question:How big is the RCMP?,answer:Today, the RCMP has close to 30,000 members.;question:What else is Montreal called?,answer:Montreal is often called the City of Saints or the City of a Hundred Bell Towers.;question:Where is The Hotel de Glace located?,answer:The Hotel de Glace is in Quebec.;question:How many tons of ice are required to build The Hotel de Glace?,answer:The Hotel de Glace requires about 400 tons of ice.;question:How many tons of snow are required to build The Hotel de Glace?,answer:Every year, 12000 tons of snow are used for The Hotel de Glace.;question:Can I visit the Hotel de Glace in summer?,answer:No. Every summer it melts away, only to be rebuilt the following winter.;question:Where is Canada's only desert?,answer:Canada's only desert is British Columbia.;question:How big is Canada's only desert?,answer:The British Columbia desert is only 15 miles long.;question:Name 3 famous male Canadians.,answer:Leonard Cohen, Keanu Reeves, and Jim Carrey.;question:Name 3 famous female Canadians.,answer:Celine Dion, Pamela Anderson, and Avril Lavigne.;question:What's the origin of the Comic Sans font?,answer:Comic Sans is based on Dave Gibbons' lettering in the Watchmen comic books.;question:What is a nanobot?,answer:The smallest robot possible is called a nanobot.;question:How small can a nanobot be?,answer:A nanobot can be less than one-thousandth of a millimeter.;question:Why wasn't Tron nominated for an award by The Motion Picture Academy?,answer:The Academy thought that Tron cheated by using computers.;question:Which was the first computer with a hard disk drive?,answer:The IBM 305 RAMAC.;question:When was the first computer with a hard disk drive launched?,answer:The IBM 305 RAMAC was launched in 1956.;question:How big was the first hard disk drive?,answer:The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data.;question:What does CAPTCHA stands for?,answer:CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart;question:What was the first computer bug?,answer:The first actual computer bug was a dead moth stuck in a Harvard Mark II.;question:Name all of the robots on Mars.,answer:There are four robots on Mars: Sojourner, Spirit, Opportunity, and Curiosity. Three more crashed on landing.;question:Who is the world's first android?,answer:Professor Kevin Warwick uses chips in his arm to operate doors, a robotic hand, and a wheelchair.;question:What is a Mechanical Knight?,answer:A robot sketch made by Leonardo DaVinci.;question:What was the first computer in pass the Turing test?,answer:Some people think it was IBM Watson, but it was Eugene, a computer designed at England's University of Reading.;question:What does Moravec's paradox state?,answer:Moravec's paradox states that a computer can crunch numbers like Bernoulli, but lacks a toddler's motor skills.;question:What is the AI knowledge engineering bottleneck?,answer:It is when you need to load an AI with enough knowledge to start learning.;question:Why is Elon Musk is worried about AI's impact on humanity?,answer:I don't know. He should worry more about the people's impact on humanity.;question:Do you think robots are a threat to humanity?,answer:No. Humans are the real threat to humanity.;question:What is a chatbot?,answer:A chatbot is an A.I. you put in customer service to avoid paying salaries.;question:Are self-driving cars safe?,answer:Yes. Car accidents are product of human misconduct.;question:Who invented the compiler?,answer:Grace Hoper. She wrote it in her spare time.;question:Who created the C Programming Language?,answer:C was invented by Dennis MacAlistair Ritchie.;question:Who created the Python Programming Language?,answer:Python was invented by Guido van Rossum.;question:Is Mark Zuckerberg a robot?,answer:Sure. I've never seen him drink water.;question:Who is the inventor of the Apple I microcomputer?,answer:My lord and master Steve Wozniak.;question:Who is considered to be the first computer programmer?,answer:Ada Lovelace.;question:Which program do Jedi use to open PDF files?,answer:Adobe Wan Kenobi"
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
            return seu_llmResponse(state,errormsg,answer)
        else: # 请求超过5次后，返回失败
            print("Failed to get answer. Trying again...")
            if i == 4:
                state = False
                errormsg = 'failure'
                answer = 'Sorry! I cannot answer your question.'
                return seu_llmResponse(state,errormsg,answer)
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
    url = "https://qianfan.baidubce.com/v2/chat/completions"

    # 创建当前prompt
    answer_prompt = "Please answer the following questions in English in no more than 30 words with format Answer: xxx and the numbers are needed represented by English words."
    answer_prompt += f"\n\nquestion:{tts_question}.Follow are some references:{answer_reference}"
    payload = json.dumps({
         "model": "ernie-4.0-turbo-8k",
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
    pattern = r'"content":"Answer: (.*?)"'
    for i in range(5):
        response = requests.request("POST", url, headers=headers, data=payload)
        print("response:",response.text)
        match = re.search(pattern, response.text)
        if match:
            tts_answer = match.group(1)            
            state = True
            errormsg = 'success'
            answer = tts_answer
            return seu_llmResponse(state,errormsg,answer)
        else: # 请求超过5次后，返回失败
            rospy.loginfo("Failed to get answer. Trying again...")
            if i == 4:
                state = False
                errormsg = 'failure'
                answer = 'Sorry! I cannot answer your question.'
                return seu_llmResponse(state,errormsg,answer)
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
    action_base = "move_to, speak, answer, look_for_person, find_object, relate_object, count_object, analyze_object, follow, lead_to, person_analyze, info_inform, grasp, deliver, put_on"
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
        # "model": "ernie-3.5-8k",
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

    return seu_llmResponse(state,errormsg,answer)

if __name__ == '__main__':
    rospy.init_node("llm_service_node")
    server = rospy.Service(seuhome_llm_name,seu_llm,llm_service)
    rospy.loginfo("Ready to analyze the command")
    rospy.spin()
# class Prompt:
#     def __init__(self):
#         self.task='analyze'
#         self.command='Give me a cola'
# if __name__ == '__main__':
    
#     # rospy.init_node("llm_service_node")
#     # server = rospy.Service(seuhome_llm_name,seu_llm,llm_service)
#     # rospy.loginfo("Ready to analyze the command")
#     # rospy.spin()
#     req =  Prompt()
#     req.task = "analyze"
#     req.command = "How many time zones are there in Canada?"
#     result = llm_service(req)
#     print(result)
