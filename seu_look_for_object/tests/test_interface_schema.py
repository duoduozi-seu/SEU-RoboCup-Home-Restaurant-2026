from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


class InterfaceSchemaTest(unittest.TestCase):
    def test_new_message_files_exist_with_core_fields(self):
        detection_msg = PACKAGE_ROOT / "msg" / "ObjectDetectionResult.msg"
        query_msg = PACKAGE_ROOT / "msg" / "ObjectQueryResult.msg"

        self.assertTrue(detection_msg.exists())
        self.assertTrue(query_msg.exists())

        detection_text = detection_msg.read_text(encoding="utf-8")
        query_text = query_msg.read_text(encoding="utf-8")

        self.assertIn("string label", detection_text)
        self.assertIn("geometry_msgs/PoseStamped pose", detection_text)
        self.assertIn("bool pose_valid", detection_text)

        self.assertIn("bool success", query_text)
        self.assertIn("string query_mode", query_text)
        self.assertIn("string resolved_label", query_text)
        self.assertIn("ObjectDetectionResult[] candidate_objects", query_text)

    def test_new_service_files_exist_with_expected_request_fields(self):
        services = {
            "FindObject.srv": ["string target_class", "string search_scope", "float32 timeout_sec"],
            "RelateObject.srv": ["string target_class", "string reference_class", "string relation"],
            "CountObject.srv": ["string target_class", "string search_scope", "float32 timeout_sec"],
            "AnalyzeObject.srv": ["string target_class", "string metric", "string search_scope"],
        }

        for filename, required_fields in services.items():
            content = read_text(f"srv/{filename}")
            for field in required_fields:
                self.assertIn(field, content)
            self.assertIn("ObjectQueryResult result", content)
