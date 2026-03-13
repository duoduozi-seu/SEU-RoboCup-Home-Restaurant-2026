#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""Prompt and Gemini config for info_inform."""

import datetime


GEMINI_API_KEY = "sk-ZzVTvgKk64GDcEFq0vf2RJzbReb6orUR9mHB0ufhwR8F2UPE"
GEMINI_BASE_URL = "https://api.viviai.cc"
GEMINI_API_VERSION = "v1beta"
GEMINI_MODEL_INFO_INFORM = "gemini-2.5-flash"
HTTP_TIMEOUT_SEC = 60
MAX_RETRY_TIMES = 5

ROBOT_NAME = "Kobe Bryant"
TEAM_NAME = "southest at home"
TEAM_COUNTRY = "China"
TEAM_AFFILIATION = "Southeast University"

INFO_INFORM_ANSWER_OPENING = "What question do you want to ask?"
INFO_INFORM_OUTPUT_RETRY_HINT = (
    "Format violation detected. Output exactly one line: Speech: <content>."
)

INFO_INFORM_OUTPUT_RULES = (
    "Strict output rules:\n"
    "1) Respond in English only.\n"
    "2) Output EXACTLY one line in this format: Speech: <content>\n"
    "3) Do NOT output anything before or after that line.\n"
    "4) Do NOT mention internal fields like ori_command, info_kind, or result_text.\n"
    "5) Use concise spoken English suitable for a service robot.\n"
    "6) When numbers appear, prefer English words when reasonable.\n"
    "7) If result_text contains factual results, prioritize those facts and do not invent missing results.\n"
    "8) If result_text is empty, rely on ori_command and known robot/team facts.\n"
)

INFO_INFORM_FEWSHOT_EXAMPLES = """
[Example 1]
ori_command: Tell me how many snacks there are on the cabinet
info_kind: query_result_count
result_text:
source_action: count_object
success: true
target_class: snack
search_scope: cabinet
count: 5
Correct output:
Speech: There are five snacks on the cabinet.

[Example 2]
ori_command: Tell me what is the biggest cleaning supply on the refrigerator
info_kind: query_result_label
result_text:
source_action: analyze_object
success: true
target_class: cleaning_supply
metric: biggest
search_scope: refrigerator
result_label: water
Correct output:
Speech: The biggest cleaning supply on the refrigerator is water.

[Example 3]
ori_command: tell me the object on the top of xuebi in the bedroom
info_kind: query_result_label
result_text:
source_action: relate_object
success: true
reference_class: xuebi
relation: top
result_label: biscuit
Correct output:
Speech: The object on top of xuebi in the bedroom is biscuit.

[Example 4]
ori_command: Tell me how many people in the kitchen are wearing white blouses
info_kind: person_count
result_text:
source_action: look_for_person
mode: count
success: true
person_feature: white_blouse
count: 3
Correct output:
Speech: There are three people wearing white blouses in the kitchen.

[Example 5]
ori_command: Tell me the gesture of the person in the bedroom
info_kind: gesture
result_text:
source_action: person_analyze
success: true
requested_feature: gesture
gesture: pointing_right
Correct output:
Speech: The person's gesture is pointing right.

[Example 6]
ori_command: Navigate to the waste basket then meet Paris and say your teams affiliation
info_kind: team_affiliation
result_text:
(empty)
Correct output:
Speech: Our team is from Southeast University in China.

[Example 7]
ori_command: Introduce yourself to the person wearing a yellow coat in the living room and follow them
info_kind: introduce
result_text:
(empty)
Correct output:
Speech: Hello, I am Kobe Bryant from Southeast University in China.

[Example 8]
ori_command: Say hello to Jules in the bathroom and follow them to the kitchen
info_kind: greet
result_text:
(empty)
Correct output:
Speech: Hello Jules, nice to meet you.

[Example 9]
ori_command: Say what day is today to the person pointing to the right in the bathroom
info_kind: today_date
result_text:
(empty)
Correct output:
Speech: Today is 2026-03-13.

[Example 10]
ori_command: Introduce yourself to the person wearing an orange jacket in the office and say your teams affiliation
info_kind: introduce and team_affiliation
result_text:
(empty)
Correct output:
Speech: Hello, I am Kobe Bryant from Southeast University in China.

[Example 11]
ori_command: Salute Jane in the kitchen and say something about yourself
info_kind: salute and introduce
result_text:
(empty)
Correct output:
Speech: Hello Jane, nice to meet you. I am Kobe Bryant from Southeast University in China.

[Example 12]
ori_command: Face Patricia at the main entrance and introduce her to all the men in the living room
info_kind: introduce
result_text:
source_action: person_analyze
success: true
requested_feature: all
person_info: with pose standing and gender is woman
Correct output:
Speech: This is Patricia. She is a woman and she is standing.
""".strip()

ANSWER_REFERENCE = (
    "question:Who's the most handsome person in Canada?,answer:I think Justin Trudeau is very handsome.;"
    "question:How many time zones are there in Canada?,answer:Canada has six time zones.;"
    "question:What's the longest street in the world?,answer:Yonge Street in Ontario is the longest street in the world.;"
    "question:What is a nanobot?,answer:A nanobot is an extremely small robot.;"
    "question:Who created the Python Programming Language?,answer:Python was invented by Guido van Rossum.;"
    "question:Who is considered to be the first computer programmer?,answer:Ada Lovelace.;"
    "question:Who invented the compiler?,answer:Grace Hopper invented the compiler.;"
    "question:What was the first computer bug?,answer:The first computer bug was a dead moth found in a computer.;"
    "question:Are self-driving cars safe?,answer:They can be safer than human drivers in many situations.;"
    "question:What is your team affiliation?,answer:My team is from Southeast University in China.;"
    "question:What is your name?,answer:My name is Kobe Bryant.;"
)


def build_background_facts(now=None):
    """Build dynamic environment facts for info_inform prompts."""
    if now is None:
        now = datetime.datetime.now()
    today = now.date()
    return (
        f"Robot name: {ROBOT_NAME}\n"
        f"Team name: {TEAM_NAME}\n"
        f"Team country: {TEAM_COUNTRY}\n"
        f"Team affiliation: {TEAM_AFFILIATION}\n"
        f"Today date: {today.isoformat()}\n"
        f"Current time: {now.strftime('%Y-%m-%d %H:%M')}\n"
        f"Day of month: {today.day}\n"
        f"Day of week: {now.strftime('%A')}\n"
    )


def build_report_prompt(info_kind, ori_command, result_text, now=None):
    """Build the general info_inform prompt."""
    result_block = (result_text or "").strip() or "(empty)"
    return (
        "You are the language module inside the robot's info_inform stage.\n"
        "The robot has already finished the previous perception or interaction step and is now standing in front of the target person.\n"
        "Your job is to decide exactly what the robot should say next.\n\n"
        f"{INFO_INFORM_OUTPUT_RULES}\n\n"
        "[Known robot and team facts]\n"
        f"{build_background_facts(now)}\n"
        "[Current inputs]\n"
        f"ori_command: {ori_command}\n"
        f"info_kind: {info_kind}\n"
        "result_text:\n"
        f"{result_block}\n\n"
        "[Few-shot examples]\n"
        f"{INFO_INFORM_FEWSHOT_EXAMPLES}\n\n"
        "[Task]\n"
        "Generate the exact speech content for the current info_inform stage.\n"
        "If result_text contains factual results, state those results naturally.\n"
        "If result_text is empty, infer the intended social speech from ori_command and info_kind.\n"
    )


def build_answer_prompt(ori_command, result_text, user_question, now=None):
    """Build the special answer prompt."""
    result_block = (result_text or "").strip() or "(empty)"
    return (
        "You are the language module inside the robot's info_inform stage.\n"
        "The robot has already completed the earlier task and has just asked the human what question they want to ask.\n"
        "Now the robot needs a concise spoken answer.\n\n"
        f"{INFO_INFORM_OUTPUT_RULES}\n\n"
        "[Known robot and team facts]\n"
        f"{build_background_facts(now)}\n"
        "[Original GPSR command]\n"
        f"{ori_command}\n\n"
        "[Previous result_text]\n"
        f"{result_block}\n\n"
        "[Human question]\n"
        f"{user_question}\n\n"
        "[Reference Q&A examples for style only]\n"
        f"{ANSWER_REFERENCE}\n\n"
        "[Task]\n"
        "Answer the human question in a concise spoken way.\n"
        "If the question is about the robot, team, date, time, or affiliation, use the known facts above.\n"
        "If the question is unrelated to the previous task, still answer it directly.\n"
    )
