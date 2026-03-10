#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
Gemini LLM 服务的统一配置文件。

说明：
1. 所有容易频繁修改的硬编码内容都集中在这里，包含：
   - 服务名、模型名、URL、API Key、路径、重试次数、正则规则、提示词模板等。
2. 业务代码（gemini_api.py / gemini_llm_service.py）只读取这里的常量，避免散落硬编码。
"""

# =========================
# ROS 服务与节点基础配置
# =========================
# ROS 服务名：你已指定为 gemini_llm_service
SEUHOME_LLM_NAME = "gemini_llm_service"

# ROS 节点名称
LLM_NODE_NAME = "llm_service_node"

# 任务模板文件路径（用于 task analyze）
PLAN_SET_PATH = "/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_llm/scripts/plan_sets/plan_set_2025.json"


# =========================
# Gemini API 基础配置
# =========================
# 你的 API Key（明文）
GEMINI_API_KEY = "sk-ZzVTvgKk64GDcEFq0vf2RJzbReb6orUR9mHB0ufhwR8F2UPE"

# API 网关地址（不要带末尾斜杠）
GEMINI_BASE_URL = "https://api.viviai.cc"

# API 版本段（用于拼接 /v1beta/models/{model}:generateContent）
GEMINI_API_VERSION = "v1beta"

# 模型配置：规划和问答可分别配置
GEMINI_MODEL_ANALYZE = "gemini-2.5-flash"
GEMINI_MODEL_ANSWER = "gemini-2.5-flash"

# HTTP 请求超时（秒）与最大重试次数
HTTP_TIMEOUT_SEC = 60
MAX_RETRY_TIMES = 5


# =========================
# Prompt / 规则常量
# =========================
# 可执行动作基元：
# 1. 物体相关动作已经拆成四个显式查询接口；
# 2. `look_for_obj` 已从动作集合中移除，避免 LLM 继续输出旧式“目标 + feature”混合接口。
ACTION_BASE = (
    "move_to, speak, answer, look_for_person, find_object, relate_object, "
    "count_object, analyze_object, follow, lead_to, person_analyze, "
    "info_inform, grasp, deliver, put_on"
)
# 使用逗号切分后逐项 strip，避免因空格不规范导致动作名合并
ACTION_BASE_LIST = [item.strip() for item in ACTION_BASE.split(",") if item.strip()]

# 任务规划主提示词：约束返回格式
ANALYZE_INSTRUCTION_PROMPT = (
    "You are a task planner for a home service robot.\n"
    "Strict output rules:\n"
    "1) Output ONLY one valid Python nested list.\n"
    "2) Do NOT output any extra words, explanations, markdown, or code fences.\n"
    "3) Do NOT use prefix like 'Answer:'.\n"
    "4) Each step must be in the form ['action', 'parameter1', ...].\n"
    "5) Action names must be chosen only from the provided action set.\n"
    "6) Use explicit object-query actions only:\n"
    "   ['find_object', target_class, search_scope]\n"
    "   ['relate_object', target_class, reference_class, relation, search_scope]\n"
    "   ['count_object', target_class, search_scope]\n"
    "   ['analyze_object', target_class, metric, search_scope]\n"
    "7) For room or place object search, first move_to the same scope, then call the object-query action. "
    "Example: ['move_to', 'kitchen'], ['count_object', 'box', 'kitchen'].\n"
    "8) If a later manipulation step uses the latest object-query result, use the "
    "token 'current_query_object' in ['grasp', ...], ['deliver', ...], or ['put_on', ...].\n"
    "9) If a later report step uses the latest object-query result, use "
    "'query_result_label' or 'query_result_count' in ['info_inform', ...].\n"
    "10) If an object-query task returns to the initial location for reporting, directly info_inform after "
    "['move_to', 'initial location']; do not add look_for_person because the asker stays at the initial location.\n"
    "11) Use underscore style for object classes and search scope names whenever possible."
)

# 任务规划的场景先验信息（与原服务保持一致）
PROMPT_KNOWN = (
    "There are something already known: "
    "foods on the desk in the bedroom(kind contain chip, biscuit, bread) and on the TV cabinet "
    "in the living room(kind contain biscuit, cookie, lays); "
    "drinks on the side tble in the living room(kind contain sprite, cola, water) or on the "
    "storage table in the kitchen(kind contain sprite, cola, water), "
    "cleaning stuffs on the dishwasher or sink in the kitchen(kind contain dishsoap, handwash, shampoo). "
    "Please note that this information may not be entirely correct, such as food items in the bedroom "
    "that do not necessarily have chip. "
    "Others: dont need count without key how many;must analyze if want to tell somebody something about "
    "somebody; you must grasp obj before deliver; you can direct move to furniture. "
    "Before any object-query action for a room or place, first use move_to with the same scope. "
    "After arriving, the object-query action performs fine-grained anchor search inside that scope. "
    "When an object query is followed by manipulation, prefer current_query_object instead of repeating or guessing "
    "the final object label. When an object query is followed by reporting, use query_result_label or "
    "query_result_count. If the robot returns to the initial location to report an object-query result, "
    "assume the asker stays at the initial location, directly info_inform, and do not add look_for_person."
)

# 问答主提示词：限制语言与输出格式
ANSWER_INSTRUCTION_PROMPT = (
    "You are a service robot developed by the team from Southeast University, China. Your name is Kobe Bryant\n"
    "Strict output rules:\n"
    "1) Respond in English only.\n"
    "2) Maximum 30 words.\n"
    "3) Output EXACTLY one line in this format: Answer: <content>\n"
    "4) Include 'Answer:' exactly once.\n"
    "5) Do NOT repeat the sentence.\n"
    "6) Do NOT output anything before or after that one line.\n"
    "7) Numbers should be written in English words when reasonable."
    "8) Convert numbers to words (e.g., 'three' not '3').\n"
)

# 二次提示（重试时加严格式约束）
PROMPT_ADD_FORMAT = (
    "Format violation detected. Regenerate with STRICT format compliance only. "
    "No extra text."
)

# 二次提示（重试时强调必须使用规定子任务）
PROMPT_ADD_MANDATED_SUBTASKS = (
    "Invalid action detected. Regenerate using ONLY the provided action set."
)

# 问答参考样例（与原服务保持一致）
ANSWER_REFERENCE = (
    "question:Who's the most handsome person in Canada?,answer:I that Justin Trudeau is very handsome.;"
    "question:How many time zones are there in Canada?,answer:Canada spans almost 10 million square km and comprises 6 time zones;"
    "question:What's the longest street in the world?,answer:Yonge Street in Ontario is the longest street in the world.;"
    "question:How long is Yonge Street in Ontario?,answer:Yonge street is almost 2,000 km, starting at Lake Ontario, and running north to the Minnesota border.;"
    "question:What's the name of the bear cub exported from Canada to the London Zoo in 1915?,answer:The bear cub was named Winnipeg. It inspired the stories of Winnie-the-Pooh.;"
    "question:Where was the Blackberry Smartphone developed?,answer:It was developed in Ontario, at Research In Motion's Waterloo offices.;"
    "question:What is the world's largest coin?,answer:The Big Nickel in Sudbury, Ontario. It is nine meters in diameter.;"
    "question:In what year was Canada invaded by the USA for the first time?,answer:The first time that the USA invaded Canada was in 1775;"
    "question:What year was Canada invaded by the USA for the second time?,answer:The USA invaded Canada a second time in 1812.;"
    "question:What country holds the record for the most gold medals at the Winter Olympics?,answer:Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics.;"
    "question:Who coined the term Beatlemania?,answer:Sandy Gardiner, a journalist of the Ottawa Journal.;"
    "question:Why is Canada named Canada?,answer:French explorers misunderstood the local native word Kanata, which means village.;"
    "question:When was The Mounted Police formed?,answer:The Mounted Police was formed in 1873.;"
    "question:When was The Royal Canadian Mounted Police formed?,answer:In 1920, when The Mounted Police merged with the Dominion Police.;"
    "question:How big is the RCMP?,answer:Today, the RCMP has close to 30,000 members.;"
    "question:What else is Montreal called?,answer:Montreal is often called the City of Saints or the City of a Hundred Bell Towers.;"
    "question:Where is The Hotel de Glace located?,answer:The Hotel de Glace is in Quebec.;"
    "question:How many tons of ice are required to build The Hotel de Glace?,answer:The Hotel de Glace requires about 400 tons of ice.;"
    "question:How many tons of snow are required to build The Hotel de Glace?,answer:Every year, 12000 tons of snow are used for The Hotel de Glace.;"
    "question:Can I visit the Hotel de Glace in summer?,answer:No. Every summer it melts away, only to be rebuilt the following winter.;"
    "question:Where is Canada's only desert?,answer:Canada's only desert is British Columbia.;"
    "question:How big is Canada's only desert?,answer:The British Columbia desert is only 15 miles long.;"
    "question:Name 3 famous male Canadians.,answer:Leonard Cohen, Keanu Reeves, and Jim Carrey.;"
    "question:Name 3 famous female Canadians.,answer:Celine Dion, Pamela Anderson, and Avril Lavigne.;"
    "question:What's the origin of the Comic Sans font?,answer:Comic Sans is based on Dave Gibbons' lettering in the Watchmen comic books.;"
    "question:What is a nanobot?,answer:The smallest robot possible is called a nanobot.;"
    "question:How small can a nanobot be?,answer:A nanobot can be less than one-thousandth of a millimeter.;"
    "question:Why wasn't Tron nominated for an award by The Motion Picture Academy?,answer:The Academy thought that Tron cheated by using computers.;"
    "question:Which was the first computer with a hard disk drive?,answer:The IBM 305 RAMAC.;"
    "question:When was the first computer with a hard disk drive launched?,answer:The IBM 305 RAMAC was launched in 1956.;"
    "question:How big was the first hard disk drive?,answer:The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data.;"
    "question:What does CAPTCHA stands for?,answer:CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart;"
    "question:What was the first computer bug?,answer:The first actual computer bug was a dead moth stuck in a Harvard Mark II.;"
    "question:Name all of the robots on Mars.,answer:There are four robots on Mars: Sojourner, Spirit, Opportunity, and Curiosity. Three more crashed on landing.;"
    "question:Who is the world's first android?,answer:Professor Kevin Warwick uses chips in his arm to operate doors, a robotic hand, and a wheelchair.;"
    "question:What is a Mechanical Knight?,answer:A robot sketch made by Leonardo DaVinci.;"
    "question:What was the first computer in pass the Turing test?,answer:Some people think it was IBM Watson, but it was Eugene, a computer designed at England's University of Reading.;"
    "question:What does Moravec's paradox state?,answer:Moravec's paradox states that a computer can crunch numbers like Bernoulli, but lacks a toddler's motor skills.;"
    "question:What is the AI knowledge engineering bottleneck?,answer:It is when you need to load an AI with enough knowledge to start learning.;"
    "question:Why is Elon Musk is worried about AI's impact on humanity?,answer:I don't know. He should worry more about the people's impact on humanity.;"
    "question:Do you think robots are a threat to humanity?,answer:No. Humans are the real threat to humanity.;"
    "question:What is a chatbot?,answer:A chatbot is an A.I. you put in customer service to avoid paying salaries.;"
    "question:Are self-driving cars safe?,answer:Yes. Car accidents are product of human misconduct.;"
    "question:Who invented the compiler?,answer:Grace Hoper. She wrote it in her spare time.;"
    "question:Who created the C Programming Language?,answer:C was invented by Dennis MacAlistair Ritchie.;"
    "question:Who created the Python Programming Language?,answer:Python was invented by Guido van Rossum.;"
    "question:Is Mark Zuckerberg a robot?,answer:Sure. I've never seen him drink water.;"
    "question:Who is the inventor of the Apple I microcomputer?,answer:My lord and master Steve Wozniak.;"
    "question:Who is considered to be the first computer programmer?,answer:Ada Lovelace.;"
    "question:Which program do Jedi use to open PDF files?,answer:Adobe Wan Kenobi"
)


# =========================
# 解析与兜底常量
# =========================
# 从模型输出中提取动作列表：例如 [[\"move_to\", \"kitchen\"], [\"speak\", \"hello\"]]
ANALYZE_PATTERN = r"\[\[.*?\]\]"

# 从模型输出中提取问答文本：使用非贪婪匹配，避免多个 Answer 段被一次性吞并
ANSWER_PATTERN = r"Answer:\s*(.*?)(?=Answer:\s*|$)"

# 失败时统一返回文案
DEFAULT_FAILURE_ANSWER = "Sorry! I cannot answer your question."
DEFAULT_FAILURE_PLAN = ""

# 额外提示：当模型未返回可解析文本时，允许把原始字符串当作兜底文本继续重试
ENABLE_RAW_TEXT_FALLBACK = True
