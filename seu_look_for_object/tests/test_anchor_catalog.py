import unittest
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SRC = PACKAGE_ROOT / "src"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

from seu_look_for_object_core.anchor_catalog import NavAnchorCatalog


class NavAnchorCatalogTest(unittest.TestCase):
    def test_groups_numeric_suffix_anchors_by_scope(self):
        raw_points = {
            "kitchen_0": {"x": 1.0, "y": 2.0, "yaw": 0.1},
            "kitchen_1": {"x": 3.0, "y": 4.0, "yaw": 0.2},
            "living_room_0": {"x": 5.0, "y": 6.0, "yaw": 0.3},
        }

        catalog = NavAnchorCatalog.from_mapping(raw_points)

        self.assertEqual(catalog.scopes(), ["kitchen", "living_room"])
        self.assertEqual(
            [anchor.name for anchor in catalog.anchors_for("kitchen")],
            ["kitchen_0", "kitchen_1"],
        )

    def test_ignores_non_numeric_suffix_anchor_names(self):
        raw_points = {
            "kitchen_a": {"x": 1.0, "y": 2.0, "yaw": 0.1},
            "kitchen_0": {"x": 3.0, "y": 4.0, "yaw": 0.2},
        }

        catalog = NavAnchorCatalog.from_mapping(raw_points)

        self.assertEqual(
            [anchor.name for anchor in catalog.anchors_for("kitchen")],
            ["kitchen_0"],
        )
