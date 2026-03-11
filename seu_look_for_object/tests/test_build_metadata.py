from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class BuildMetadataTest(unittest.TestCase):
    def test_cmake_declares_new_messages_and_services_without_python_package_setup(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

        self.assertNotIn("catkin_python_setup()", cmake)
        self.assertIn("ObjectDetectionResult.msg", cmake)
        self.assertIn("ObjectQueryResult.msg", cmake)
        self.assertIn("FindObject.srv", cmake)
        self.assertIn("RelateObject.srv", cmake)
        self.assertIn("CountObject.srv", cmake)
        self.assertIn("AnalyzeObject.srv", cmake)
        self.assertIn("catkin_install_python(PROGRAMS", cmake)
        self.assertIn("scripts/object_query_server.py", cmake)

    def test_package_declares_runtime_dependencies_for_find_runtime(self):
        package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

        for dependency in [
            "actionlib",
            "geometry_msgs",
            "message_runtime",
            "sensor_msgs",
            "std_msgs",
            "tf2_ros",
            "visualization_msgs",
        ]:
            self.assertIn(dependency, package_xml)

        self.assertIn("<license>MIT</license>", package_xml)

    def test_legacy_python_package_layout_is_removed(self):
        self.assertFalse((PACKAGE_ROOT / "setup.py").exists())
        self.assertFalse((PACKAGE_ROOT / "src" / "seu_look_for_object_core").exists())
