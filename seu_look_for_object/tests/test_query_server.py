import unittest
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SRC = PACKAGE_ROOT / "src"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

from seu_look_for_object_core.query_server import ObjectQueryService


class QueryServerTest(unittest.TestCase):
    def test_find_handler_returns_not_implemented_without_perception_backend(self):
        service = ObjectQueryService(anchor_catalog=None)

        result = service.handle_find(
            target_class="orange_juice",
            search_scope="kitchen",
            timeout_sec=20.0,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.status_code, "NOT_IMPLEMENTED")
        self.assertIn("感知", result.message)

    def test_relate_handler_requires_reference_and_target_information(self):
        service = ObjectQueryService(anchor_catalog=None)

        result = service.handle_relate(
            target_class="snack",
            reference_class="lays",
            relation="BEHIND",
            search_scope="kitchen",
            timeout_sec=20.0,
        )

        self.assertEqual(result.query_mode, "RELATE")
        self.assertEqual(result.target_class, "snack")
        self.assertEqual(result.reference_class, "lays")
