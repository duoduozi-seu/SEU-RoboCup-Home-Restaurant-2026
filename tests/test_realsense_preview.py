import importlib.util
import sys
import types
import unittest
from pathlib import Path

import numpy as np
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "realsense_test.py"


def load_module():
    fake_rs = types.SimpleNamespace()
    fake_cv2 = types.SimpleNamespace(
        WINDOW_AUTOSIZE=1,
        FONT_HERSHEY_SIMPLEX=0,
        namedWindow=lambda *args, **kwargs: None,
        putText=lambda *args, **kwargs: None,
        imshow=lambda *args, **kwargs: None,
        waitKey=lambda *args, **kwargs: -1,
        destroyAllWindows=lambda *args, **kwargs: None,
    )
    spec = importlib.util.spec_from_file_location("realsense_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    with patch.dict(sys.modules, {"pyrealsense2": fake_rs, "cv2": fake_cv2}):
        spec.loader.exec_module(module)
    return module


class FakeFrame:
    def __init__(self, data):
        self._data = data

    def get_data(self):
        return self._data


class RealSensePreviewTest(unittest.TestCase):
    def test_default_resolution_is_the_largest_preset(self):
        module = load_module()

        self.assertEqual(module.DEFAULT_RESOLUTION, (1920, 1080))
        self.assertEqual(module.DEFAULT_RESOLUTION, max(module.RESOLUTION_OPTIONS, key=lambda item: item[0] * item[1]))

    def test_color_frame_to_bgr_returns_none_for_missing_frame(self):
        module = load_module()

        self.assertIsNone(module.color_frame_to_bgr(None))

    def test_color_frame_to_bgr_converts_valid_frame(self):
        module = load_module()
        image = np.arange(27, dtype=np.uint8).reshape(3, 3, 3)

        converted = module.color_frame_to_bgr(FakeFrame(image))

        self.assertTrue(np.array_equal(converted, image))


if __name__ == "__main__":
    unittest.main()
