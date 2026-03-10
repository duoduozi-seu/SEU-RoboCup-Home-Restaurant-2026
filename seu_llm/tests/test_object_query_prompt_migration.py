import importlib.util
import json
import sys
import unittest
from pathlib import Path


LLM_ROOT = Path("/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_llm")
CONFIGS_PATH = LLM_ROOT / "scripts" / "configs.py"
PLAN_SET_PATH = LLM_ROOT / "scripts" / "plan_sets" / "plan_set_2025.json"


def _load_configs_module():
    sys.path.insert(0, str(CONFIGS_PATH.parent))
    spec = importlib.util.spec_from_file_location("seu_llm_configs", CONFIGS_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ObjectQueryPromptMigrationTest(unittest.TestCase):
    """约束 LLM 规划层已经迁移到四个显式物体查询动作。"""

    @classmethod
    def setUpClass(cls):
        cls.configs = _load_configs_module()
        cls.plan_examples = json.loads(PLAN_SET_PATH.read_text(encoding="utf-8"))

    def test_action_base_uses_explicit_object_query_actions(self):
        action_base_list = self.configs.ACTION_BASE_LIST
        self.assertNotIn("look_for_obj", action_base_list)
        self.assertIn("find_object", action_base_list)
        self.assertIn("relate_object", action_base_list)
        self.assertIn("count_object", action_base_list)
        self.assertIn("analyze_object", action_base_list)

    def test_prompt_mentions_new_object_query_interfaces(self):
        prompt_text = self.configs.ANALYZE_INSTRUCTION_PROMPT
        self.assertIn("find_object", prompt_text)
        self.assertIn("relate_object", prompt_text)
        self.assertIn("count_object", prompt_text)
        self.assertIn("analyze_object", prompt_text)
        self.assertIn("first move_to", prompt_text)
        self.assertIn("current_query_object", prompt_text)
        self.assertIn("query_result_label", prompt_text)
        self.assertIn("query_result_count", prompt_text)
        self.assertIn("directly info_inform", prompt_text)
        self.assertIn("do not add look_for_person", prompt_text)

    def test_plan_examples_do_not_use_legacy_object_query_action(self):
        for command, action_text in self.plan_examples.items():
            self.assertNotIn("look_for_obj", action_text, msg=command)

    def test_plan_examples_use_new_result_tokens(self):
        serialized = json.dumps(self.plan_examples, ensure_ascii=False)
        self.assertNotIn("obj_name", serialized)
        self.assertNotIn("obj_count", serialized)
        self.assertIn("current_query_object", serialized)
        self.assertIn("query_result_label", serialized)
        self.assertIn("query_result_count", serialized)

    def test_plan_examples_prepend_move_to_before_object_query(self):
        object_query_actions = {
            "find_object",
            "relate_object",
            "count_object",
            "analyze_object",
        }
        for command, action_text in self.plan_examples.items():
            actions = eval(action_text)
            for index, action in enumerate(actions):
                if action[0] in object_query_actions:
                    self.assertGreater(index, 0, msg=command)
                    self.assertEqual(actions[index - 1][0], "move_to", msg=command)
                    self.assertEqual(actions[index - 1][1], action[-1], msg=command)

    def test_box_count_example_exists_for_room_search(self):
        self.assertIn(
            "Tell me how many boxes there are in the kitchen",
            self.plan_examples,
        )
        self.assertEqual(
            self.plan_examples["Tell me how many boxes there are in the kitchen"],
            "[['move_to', 'kitchen'], ['count_object', 'box', 'kitchen'], ['move_to', 'initial location'], ['info_inform', 'query_result_count']]",
        )

    def test_object_report_examples_do_not_search_for_asker_again(self):
        object_report_commands = {
            "Tell me how many snacks there are on the cabinet",
            "Tell me how many boxes there are in the kitchen",
            "Tell me what is the biggest cleaning supply on the refrigerator",
            "Tell me what is the biggest thing in the living room",
            "tell me the object on the top of xuebi in the bedroom",
            "Tell me what is the heaviest snack on the pantry",
        }
        for command in object_report_commands:
            actions = eval(self.plan_examples[command])
            self.assertEqual(actions[-2], ["move_to", "initial location"], msg=command)
            self.assertEqual(actions[-1][0], "info_inform", msg=command)
            self.assertNotIn(["look_for_person", "me", "nocount"], actions, msg=command)

    def test_object_report_prompt_mentions_waiting_asker_rule(self):
        prompt_text = self.configs.PROMPT_KNOWN
        self.assertIn("asker stays at the initial location", prompt_text)
        self.assertIn("directly info_inform", prompt_text)
        self.assertIn("do not add look_for_person", prompt_text)


if __name__ == "__main__":
    unittest.main()
