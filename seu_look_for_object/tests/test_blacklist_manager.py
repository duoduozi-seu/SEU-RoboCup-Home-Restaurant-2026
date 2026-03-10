import unittest
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SRC = PACKAGE_ROOT / "src"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

from seu_look_for_object_core.blacklist_manager import BlacklistManager


class BlacklistManagerTest(unittest.TestCase):
    def test_blocks_points_inside_blacklisted_radius(self):
        manager = BlacklistManager(default_radius=0.4)
        manager.add_zone(1.0, 1.0)

        self.assertTrue(manager.contains(1.1, 1.1))
        self.assertFalse(manager.contains(1.6, 1.6))

    def test_allows_custom_radius_for_specific_zone(self):
        manager = BlacklistManager(default_radius=0.4)
        manager.add_zone(0.0, 0.0, radius=1.0)

        self.assertTrue(manager.contains(0.6, 0.2))
        self.assertFalse(manager.contains(1.2, 0.0))
