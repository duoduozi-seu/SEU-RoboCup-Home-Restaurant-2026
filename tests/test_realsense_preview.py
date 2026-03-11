import importlib.util
import sys
import types
import unittest
from pathlib import Path

import numpy as np
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "realsense_test.py"


def load_module(fake_rs=None, fake_cv2=None):
    fake_rs = fake_rs or types.SimpleNamespace()
    fake_cv2 = fake_cv2 or types.SimpleNamespace(
        WINDOW_AUTOSIZE=1,
        FONT_HERSHEY_SIMPLEX=0,
        COLORMAP_JET=2,
        namedWindow=lambda *args, **kwargs: None,
        putText=lambda *args, **kwargs: None,
        imshow=lambda *args, **kwargs: None,
        waitKey=lambda *args, **kwargs: -1,
        destroyAllWindows=lambda *args, **kwargs: None,
        convertScaleAbs=lambda image, alpha=1.0: np.clip(np.abs(image) * alpha, 0, 255).astype(np.uint8),
        applyColorMap=lambda image, colormap: np.repeat(image[..., None], 3, axis=2),
        resize=lambda image, size: np.zeros((size[1], size[0], image.shape[2]), dtype=image.dtype),
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
    def test_default_resolution_is_the_largest_common_aligned_preset(self):
        module = load_module()

        self.assertEqual(module.DEFAULT_RESOLUTION, (1280, 720))
        self.assertEqual(module.DEFAULT_RESOLUTION, max(module.RESOLUTION_OPTIONS, key=lambda item: item[0] * item[1]))

    def test_aligned_profile_options_cover_common_same_resolution_pairs(self):
        module = load_module()

        expected_profiles = [
            {"color": (1280, 720), "depth": (1280, 720), "fps": 30},
            {"color": (848, 480), "depth": (848, 480), "fps": 30},
            {"color": (640, 480), "depth": (640, 480), "fps": 30},
            {"color": (640, 360), "depth": (640, 360), "fps": 30},
            {"color": (424, 240), "depth": (424, 240), "fps": 30},
        ]

        self.assertTrue(all(profile in module.ALIGNED_PROFILE_OPTIONS for profile in expected_profiles))

    def test_build_stream_config_enables_both_color_and_depth_streams(self):
        fake_config = types.SimpleNamespace(calls=[])

        def enable_stream(*args):
            fake_config.calls.append(args)

        fake_config.enable_stream = enable_stream
        fake_rs = types.SimpleNamespace(
            config=lambda: fake_config,
            stream=types.SimpleNamespace(depth="depth", color="color"),
            format=types.SimpleNamespace(z16="z16", bgr8="bgr8"),
        )

        module = load_module(fake_rs=fake_rs)

        result = module.build_stream_config(fake_rs, module.DEFAULT_PROFILE)

        self.assertIs(result, fake_config)
        self.assertEqual(
            fake_config.calls,
            [
                ("depth", 1280, 720, "z16", 30),
                ("color", 1280, 720, "bgr8", 30),
            ],
        )

    def test_color_frame_to_bgr_returns_none_for_missing_frame(self):
        module = load_module()

        self.assertIsNone(module.color_frame_to_bgr(None))

    def test_color_frame_to_bgr_converts_valid_frame(self):
        module = load_module()
        image = np.arange(27, dtype=np.uint8).reshape(3, 3, 3)

        converted = module.color_frame_to_bgr(FakeFrame(image))

        self.assertTrue(np.array_equal(converted, image))

    def test_depth_frame_to_array_returns_none_for_missing_frame(self):
        module = load_module()

        self.assertIsNone(module.depth_frame_to_array(None))

    def test_depth_frame_to_array_converts_valid_frame(self):
        module = load_module()
        image = np.arange(12, dtype=np.uint16).reshape(3, 4)

        converted = module.depth_frame_to_array(FakeFrame(image))

        self.assertTrue(np.array_equal(converted, image))


if __name__ == "__main__":
    unittest.main()
