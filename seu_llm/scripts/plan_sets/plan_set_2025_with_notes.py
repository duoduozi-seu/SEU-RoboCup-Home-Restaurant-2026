import json
action_dict = {
    # 去某地找某人告知信息
    "Navigate to the waste basket then meet Paris and say your teams affiliation":"[['move_to', 'waste_basket'], ['look_for_person', 'Paris', 'nocount'], ['info_inform', 'team_affiliation']]",
    # 去某地找某人回答问题
    "Meet Morgan in the office and answer a question":"[['move_to', 'office'], ['look_for_person', 'Morgan', 'nocount'], ['info_inform', 'answer']]",
    # 去某地找某人跟随到某地
    "Find a person pointing to the left in the office and follow them to the kitchen":"[['move_to', 'office'], ['look_for_person', 'pointing_left', 'nocount'], ['follow', 'kitchen']]",
    # 去某地找某人跟随无明确目标
    "Go to the shelf then meet Angel and follow them":"[['move_to', 'shelf'], ['look_for_person', 'Angel', 'nocount'], ['follow', 'none']]",
    # 去某地找某人指引到某地
    "Go to the kitchen then meet Adel and take them to the desk":"[['move_to', 'kitchen'], ['look_for_person', 'Adel', 'nocount'], ['lead_to', 'desk']]",
    # 去某地找某物拿某物放某地
    "Go to the kitchen then look for a fruit and grasp it and put it on the sink":"[['move_to','kitchen'],['look_for_obj','fruit','none'],['grasp','fruit'],['move_to','sink'],['put_on','fruit','sink']]",
    # 去某地找某物给我
    "Go to the bathroom then look for an orange juice and get it and give it to me":"[['move_to', 'bathroom'], ['look_for_obj', 'orange_juice','none'], ['grasp', 'orange_juice'], ['move_to', 'initial location'], ['look_for_person', 'me'], ['deliver', 'orange_juice']]",
    # 去某地找某物给某地的某人
    "Get an orange from the kitchen table and bring it to the standing person in the bathroom":"[['move_to', 'kitchen_table'], ['look_for_obj', 'orange','none'], ['grasp', 'orange'], ['move_to', 'bathroom'], ['look_for_person', 'standing', 'nocount'], ['deliver', 'orange']]",
    "Please give me the object behind the lays from the storage table": "[['move_to', 'storage table'], ['look_for_obj', 'lays', 'behind'], ['grasp', 'obj_name'], ['move_to', 'initial location'], ['look_for_person', 'me', 'nocount'], ['deliver', 'obj_name']]",
    # 告诉我某地某物的数量
    "Tell me how many snacks there are on the cabinet":"[['move_to', 'cabinet'], ['look_for_obj', 'snack','count'],['move_to', 'initial location'], ['look_for_person', 'me'], ['info_inform', 'obj_count']]",
    # 告诉我某地怎样的人的数量
    "Tell me how many persons raising their right arm are in the living room":"[['move_to', 'living room'], ['look_for_person, 'raising_right','count'], ['move_to','initial location'], ['look_for_person','me', 'nocount'], ['info_inform','person_count']]",
    "Tell me how many people in the living room are wearing black sweaters":"[['move_to', 'living room'], ['look_for_person', 'black_sweater', 'count'], ['move_to','initial location'], ['look_for_person','me', 'nocount'], ['info_inform', 'person_count']]",
    # 告诉我某地的某人的某些信息
    "Tell me the gesture of the person in the bedroom": "[['move_to', 'bedroom'], ['look_for_person', 'any', 'nocount'], ['person_analyze', 'gesture'],  ['move_to','initial location'], ['look_for_person','me', 'nocount'], ['info_inform', 'gesture']]",
    # 告诉我某地的某物的某些信息
    "Tell me what is the biggest cleaning supply on the refrigerator":"[['move_to', 'refrigerator'], ['look_for_obj', 'cleaning supply','biggest'], ['move_to','initial location'], ['look_for_person','me', 'nocount'], ['info_inform', 'obj_name']]",
    "Tell me what is the biggest thing in the living room":"[['move_to', 'living room'], ['look_for_obj', 'any','biggest'], ['move_to','initial location'], ['look_for_person','me', 'nocount'], ['info_inform', 'obj_name']]",
    "tell me the object on the top of xuebi in the bedroom": "[['move_to', 'bedroom'], ['look_for_obj', 'xuebi','top'], ['move_to', 'initial location'], ['look_for_person', 'me', 'nocount'], ['info_inform', 'obj_name']]",
    # 跟某衣着的人打招呼然后各种动作
    "Introduce yourself to the person wearing a yellow coat in the living room and follow them":"[['move_to', 'living room'], ['look_for_person', 'yellow_coat', 'nocount'], ['info_inform', 'introduce'], ['follow', 'none']]",
    # 跟某名字的人打招呼然后各种动作
    "Say hello to Jules in the bathroom and follow them to the kitchen": "[['move_to', 'bathroom'], ['look_for_person', 'Jules', 'nocount'], ['info_inform', 'greet'], ['follow', 'kitchen']]",
    # 去某地告诉某人某些信息
    "Say what day is today to the person pointing to the right in the bathroom":"[['move_to', 'bathroom'], ['look_for_person', 'pointing_right', 'nocount'], ['info_inform', 'today_date']]",
    # 去某地回答怎样的某人的问题
    "Answer the quiz of the person pointing to the left in the living room": "[['move_to', 'living room'], ['look_for_person', 'pointing_left', 'nocount'], ['info_inform', 'answer']]",
    # 更随某地的某人去某地
    "Follow Jane from the bedside table to the bathroom":"[['move_to', 'bedside_table'], ['look_for_person', 'Jane', 'nocount'], ['follow', 'bathroom']]",
    # 引导某地的某人（姓名）去某地
    "Escort Charlie from the storage rack to the living room":"[['move_to', 'storage_rack'], ['look_for_person', 'Charlie', 'nocount'], ['lead_to', 'living room']]",
    # 引导某地的某人（手势和坐姿）去某地
    "Take the person pointing to the right from the refrigerator to the desk":"[['move_to', 'refrigerator'], ['look_for_person', 'pointing_right', 'nocount'], ['lead_to', 'desk']]",
    # 引导某地的某人（衣着）去某地
    "Take the person wearing a red shirt from the potted plant to the living room":"[['move_to', 'potted_plant'], ['look_for_person', 'red_shirt', 'nocount'], ['lead_to', 'living room']]",
    "Lead me to the living room":"[['move_to','initial location'], ['look_for_person','me', 'nocount'],['lead_to', 'living room']]",
    # 给我某地的某物
    "Give me a chocolate jello from the shelf":"[['move_to', 'shelf'], ['look_for_obj', 'chocolate jello','none'], ['grasp', 'chocolate jello'], ['move_to','initial location'], ['look_for_person','me', 'nocount'],['deliver', 'orange']]",
    # 告诉我某地怎么样的某物是什么
    "Tell me what is the heaviest snack on the pantry": "[['move_to', 'pantry'], ['look_for_obj', 'snack','heaviest'],  ['move_to','initial location'], ['look_for_person','me', 'nocount'], ['info_inform', 'obj_name']]",
    # 给某地某着装的人打招呼
    "Introduce yourself to the person wearing an orange jacket in the office and say your teams affiliation":"[['move_to', 'office'], ['look_for_person', 'orange_jacket', 'nocount'], ['info_inform', 'introduce'], ['info_inform', 'team_affiliation']]",
    # 给某地某名字的人打招呼
    "Salute Jane in the kitchen and say something about yourself":"[['move_to', 'kitchen'], ['look_for_person', 'Jane', 'nocount'], ['info_inform', 'salute'], ['info_inform', 'introduce']]",
    # 在某地找到某人然后再去某地找他
    "Meet Jane at the entrance then find them in the bedroom":"[['move_to','entrance'],['look_for_person','Jane','nocount'],['move_to','bedroom'],['look_for_person','Jane','nocount']]",
    # 统计某地某衣着人的数量
    "Tell me how many people in the kitchen are wearing white blouses":"[['move_to', 'kitchen'], ['look_for_person', 'white_blouse', 'count'], ['move_to','initial location'], ['look_for_person','me', 'nocount'], ['info_inform', 'person_count']]",
    # 告诉某地某人的某地2某人2的信息
    "Tell Mary in living room how many people in the bedroom are wearing white shirt": "[['move_to', 'bedroom'], ['look_for_person', 'white_shirt', 'count'], ['move_to', 'living room'], ['look_for_person', 'Mary', 'nocount'], ['info_inform', 'person_count']]",
    "Face Patricia at the main entrance and introduce her to all the men in the living room":"[['move_to', 'main entrance'], ['look_for_person', 'Patricia', 'nocount'], ['person_analyze', 'all'],['move_to', 'living room'], ['look_for_person', 'male', 'nocount'], ['info_inform', 'introduce']]",
    # 跟随某地的某人
    "Follow the person raising their right arm at the potted plant": "[['move_to', 'potted_plant'], ['look_for_person', 'raising_right', 'nocount'], ['follow', 'none']]",
    # 直接回答问题
    "Who is the most handsome person in Canada":"[['answer','Who is the most handsome person in Canada']]",
    # 其他
    "Go to the washroom pick up a paper and give it to John":"[['move_to', 'washroom'], ['look_for_obj', 'paper','none'], ['grasp', 'paper'], ['look_for_person', 'John'], ['deliver', 'paper']]",
    "Get an orange juice from the sink and put orange juice on the bookshelf":"[['move_to', 'sink'], ['look_for_obj', 'orange_juice','none'], ['grasp', 'orange_juice'], ['move_to', 'bookshelf'], ['put_on', 'orange_juice', 'bookshelf']]",
    #信息不完全
    "please give me a drink":"[['move_to', 'side_table'], ['look_for_obj', 'drink','none'], ['grasp', 'drink'], ['move_to', 'initial location'], ['look_for_person', 'me', 'nocount'], ['deliver', 'drink']]",
    "please give me a food": "[['move_to', 'desk', 'bedroom'], ['look_for_obj', 'food','none'], ['grasp', 'food'], ['move_to', 'initial location'], ['look_for_person', 'me', 'nocount'], ['deliver', 'food']]",
    "please give me a cleaning stuff":"[['move_to', 'dishwasher'], ['look_for_obj', 'cleaning stuff','none'], ['grasp', 'cleaning stuff'], ['move_to', 'initial location'], ['look_for_person', 'me', 'nocount'], ['deliver', 'cleaning stuff']]",
}

# 将字典转换为格式化的JSON字符串
json_data = json.dumps(action_dict, indent=4, ensure_ascii=False)

# 将JSON数据保存到文件
with open('/home/syw/code/robocup_nation/2024code/GPSR/src/seu_llm/scripts/plan_sets/plan_set_2025.json', 'w', encoding='utf-8') as file:
    json.dump(action_dict, file, indent=4, ensure_ascii=False)

print("JSON数据已成功保存到 action_commands.json")