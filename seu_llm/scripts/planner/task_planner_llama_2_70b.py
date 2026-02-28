import requests
import json
import re

# 注意修改plan_set.json的路径！
plan_set_path = "/home/syw/robocup/GPSR_code/seu_llm/scripts/plan_sets/plan_set.json"


def get_access_token():        
    """
    使用 API Key，Secret Key 获取access_token，替换下列示例中的应用API Key、应用Secret Key
    """
        
    url = "https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=[应用API Key]&client_secret=[应用Secret Key]"
    
    payload = json.dumps("")
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json'
    }
    
    response = requests.request("POST", url, headers=headers, data=payload)
    return response.json().get("access_token")


# 自定义测试函数，实际使用时需要替换为自己的函数
def move_to(obj):
    print(f"move_to({obj}) successfully")
def look_for_obj(obj):
    print(f"look_for_obj({obj}) successfully")
def look_for_person(obj):
    print(f"look_for_person({obj}) successfully")
def follow(obj):
    print(f"follow({obj}) successfully")
def grasp(obj):
    print(f"grasp({obj}) successfully")
def pass_to(obj):
    print(f"pass_to({obj}) successfully")
def speak(tts_answer):
    print(tts_answer)
    print(f"speak() successfully") 
def answer(str):
    # 语音播报：Hi, how can I help you?
    speak("Hi, how can I help you?")
    # 接收语音问题输入
    tts_question = "Are self-driving cars safe?"
    print(tts_question)
    tts_answer = get_answer(tts_question)
    speak(tts_answer)
    return tts_answer


def get_answer(tts_question):
    url = "https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/llama_2_70b?access_token=" + get_access_token()
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
    response = requests.request("POST", url, headers=headers, data=payload)
    # 将回答转换为文本tts_answer
    tts_answer  = response.text.split('"result":"')[1].split('","is_truncated"')[0]
    return tts_answer


def task_plan(task):
    url = "https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/llama_2_70b?access_token=" + get_access_token()
    
    # 定义基元动作
    action_base = "move_to, look_for_obj, look_for_person, follow, grasp, pass_to, speak, answer"
    prompt = f"from actions import {action_base}. Note that pass_to indicates that the robot releases the gripping jaws and drops the picked up item. Note that the follow function is called when the robot needs to accompany, escort, or follow someone to a place."
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
    prompt += "\n\nAnswer the following question with reference to the examples in the form of a nested python list without any modifiers. Be careful not to abbreviate words in your answer. Your response is expected to be in the following format: 'Answer: [['actions', 'parameters'], ..., ['actions', 'parameters']]'."

    # 添加当前任务
    print("Task:",task)

    # 创建当前prompt
    curr_prompt = f"{prompt}\n\n{task}\n\t"

    payload = json.dumps({
        "messages": [
            {
                "role": "user",
                "content": curr_prompt
            }
        ]
    })
    headers = {
        'Content-Type': 'application/json'
    }
    
    pattern = r"\[\[.*?\]\]"
    prompt_add1 = "Please note the format of the answer!"
    prompt_add2 = "Please note that mandated subtasks need to be used to complete task planning"
    
    for i in range(5):
        response = requests.request("POST", url, headers=headers, data=payload)
        print(response.text)
        match = re.search(pattern, response.text)
        if match:
            action_str = match.group()
            # 将匹配到的字符串转换为函数列表
            action_lst = eval(action_str)
            # 如果规划的子任务不在任务模板中，则提示用户重新回答
            if not all([action in action_base_lst for action, _ in action_lst]):
                print("Error: Planned subtasks are not executable. Trying again...")
                curr_prompt = f"{prompt}\n\n{task}\n\n{prompt_add2}"
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
                break          
        else:
            print("Error: Failed to store result. Trying again...")
            # 添加提示词：请注意回答的格式!
            curr_prompt = f"{prompt}\n\n{task}\n\n{prompt_add1}"
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
            print("Error: Failed to store result after 5 attempts. Task planning failed.")
            action_str = ""
            break

    return action_str


def main():
    # 在这里输入任务
    action_str = task_plan("Bring me the object on top of the potato chips from the counter")
    if(action_str):
        action_lst = eval(action_str)
        print(action_lst)
        # 执行规划的子任务
        for func_name, *args in action_lst:
                func = globals()[func_name]
                func(*args)
    


if __name__ == '__main__':
    main()