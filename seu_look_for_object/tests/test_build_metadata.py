from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class BuildMetadataTest(unittest.TestCase):
    def test_cmake_declares_new_messages_and_services(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

        self.assertIn("catkin_python_setup()", cmake)
        self.assertIn("ObjectDetectionResult.msg", cmake)
        self.assertIn("ObjectQueryResult.msg", cmake)
        self.assertIn("FindObject.srv", cmake)
        self.assertIn("RelateObject.srv", cmake)
        self.assertIn("CountObject.srv", cmake)
        self.assertIn("AnalyzeObject.srv", cmake)
        self.assertIn("catkin_install_python(PROGRAMS", cmake)

    def test_package_declares_runtime_dependencies_for_new_scheme_b_layout(self):
        package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

        for dependency in [
            "geometry_msgs",
            "message_runtime",
            "std_msgs",
        ]:
            self.assertIn(dependency, package_xml)

        self.assertIn("<license>MIT</license>", package_xml)
