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
        names = ["kitchen_0", "kitchen_1", "living_room_0", "Kitchen", "StartPoint"]

        catalog = NavAnchorCatalog.from_name_list(names)

        self.assertEqual(catalog.scopes(), ["kitchen", "living_room"])
        self.assertEqual(
            [anchor.name for anchor in catalog.anchors_for("kitchen")],
            ["kitchen_0", "kitchen_1"],
        )

    def test_ignores_non_numeric_suffix_anchor_names(self):
        names = ["kitchen_a", "kitchen_0"]

        catalog = NavAnchorCatalog.from_name_list(names)

        self.assertEqual(
            [anchor.name for anchor in catalog.anchors_for("kitchen")],
            ["kitchen_0"],
        )

    def test_anchor_names_for_convenience_method(self):
        names = ["Kitchen_0", "Kitchen_1", "Bedroom_0"]

        catalog = NavAnchorCatalog.from_name_list(names)

        self.assertEqual(catalog.anchor_names_for("Kitchen"), ["Kitchen_0", "Kitchen_1"])
        self.assertEqual(catalog.anchor_names_for("Bedroom"), ["Bedroom_0"])
        self.assertEqual(catalog.anchor_names_for("NonExistent"), [])

