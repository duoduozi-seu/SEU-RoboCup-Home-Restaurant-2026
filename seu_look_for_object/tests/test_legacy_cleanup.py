from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class LegacyCleanupTest(unittest.TestCase):
    def test_legacy_yolo_runtime_files_are_removed(self):
        self.assertFalse((PACKAGE_ROOT / "scripts" / "yolov7" / "find_object_camera_2025.py").exists())
        self.assertFalse((PACKAGE_ROOT / "scripts" / "yolov7" / "yolo.py").exists())
