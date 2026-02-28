# 该代码用于调用讯飞星火大模型API进行任务规划及问题回答(未和smach整合)

# 使用时，需要将spark_speaker.py中的config_path参数修改为配置文件(config.json)的绝对路径，
# task_planner_xf.py中的plan_set_root参数修改为喂给大模型的例子(plan_set.json)的绝对路径

from spark_speaker import SparkSpeaker
import requests
import json
import re
import time

# plan_set_root 为喂给大模型的例子(plan_set8.json)的绝对路径，需要自行修改
plan_set_root = '/home/syw/robocup/GPSR_code/seu_llm/scripts/plan_sets/plan_set.json'

def task_plan(task):
    # 定义基元动作
    action_base = "move_to, look_for_obj, look_for_person, follow, grasp, pass_to, speak, answer"
    prompt = f"from actions import {action_base}"
    # 将action_base转换为列表形式
    action_base_lst = action_base.split(", ")
    # print(action_base_lst)

    # 加载任务模板
    with open(plan_set_root, 'r') as f:
        tmp = json.load(f)
        prompt_egs = {}
        for k, v in tmp.items():
            prompt_egs[k] = v
        prompt += f"\n\nexamples = {prompt_egs}"
    # print("prompt:", prompt)

    # 加载任务提示
    prompt += "\n\nAnswer the following question with reference to the examples in the form of a nested python list without any modifiers. Your response is expected to be in the following format: 'Answer: [['actions', 'parameters'], ..., ['actions', 'parameters']]'."
    speaker = SparkSpeaker(prompt)

    # 创建当前prompt
    curr_prompt = f"{prompt}\n\n{task}\n\t"
    # print("curr_prompt:",curr_prompt)

    pattern = r"\[\[.*?\]\]"
    prompt_add1 = "Please note the format of the answer!"
    prompt_add2 = "Please note that mandated subtasks need to be used to complete task planning"

    start_time = time.time()  # 记录开始时间
    for i in range(5):
        response = speaker.ask(curr_prompt)
        # print(response)
        match = re.search(pattern,  response)
        # print(i)
        if match:
            action_str = match.group()
            # print(action_str)
            # 将匹配到的字符串转换为函数列表
            action_lst = eval(match.group())
            # 如果规划的子任务不在任务模板中，则提示用户重新回答
            if not all([action in action_base_lst for action, _ in action_lst]):
                print("Error: Planned subtasks are not executable. Trying again...")
                curr_prompt = f"{prompt}\n\n{task}\n\n{prompt_add2}"
                # print(curr_prompt)
            else:
                print("Task plan successfully!")
                break
        else:
            print("Error: Failed to store result. Trying again...")
            # 添加提示词：请注意回答的格式!
            curr_prompt = f"{prompt}\n\n{task}\n\n{prompt_add1}"

        # print(i)
        if i == 4:
            # 请求超过5次后，返回失败
            print("Error: Failed to store result after 5 attempts. Task planning failed.")
            action_str = ""
            break

    end_time = time.time()  # 记录结束时间
    elapsed_time = end_time - start_time  # 计算经过的时间
    print("执行时间：", elapsed_time, "秒")
    return action_str

if __name__ == '__main__':
    action_str = task_plan("Meet Elizabeth at the bookcase and accompany her to the couch")

    print(action_str)
