import unittest
from pathlib import Path


SMACH_MAIN_PATH = Path(
    "/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_smach_main/scripts/gpsr_smach.py"
)


class ObjectQueryStateMigrationTest(unittest.TestCase):
    """静态检查主状态机是否已迁移到四服务物体查询链路。"""

    @classmethod
    def setUpClass(cls):
        cls.source_text = SMACH_MAIN_PATH.read_text(encoding="utf-8")

    def test_new_object_query_states_exist(self):
        self.assertIn("class FindObjectState", self.source_text)
        self.assertIn("class RelateObjectState", self.source_text)
        self.assertIn("class CountObjectState", self.source_text)
        self.assertIn("class AnalyzeObjectState", self.source_text)

    def test_legacy_look_for_obj_state_is_removed(self):
        self.assertNotIn("class LookForObjState", self.source_text)
        self.assertNotIn("elif action == 'look_for_obj'", self.source_text)

    def test_new_services_and_result_userdata_are_used(self):
        self.assertIn("from seu_look_for_object.srv import", self.source_text)
        self.assertIn("object_query_result", self.source_text)
        self.assertIn("selected_object", self.source_text)
        self.assertIn("query_result_label", self.source_text)
        self.assertIn("query_result_count", self.source_text)


if __name__ == "__main__":
    unittest.main()
