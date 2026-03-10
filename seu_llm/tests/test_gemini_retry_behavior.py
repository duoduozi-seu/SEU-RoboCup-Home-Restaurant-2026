import importlib.util
import sys
import types
import unittest
from pathlib import Path


SCRIPTS_DIR = Path("/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_llm/scripts")
SERVICE_PATH = SCRIPTS_DIR / "gemini_llm_service.py"


class _FakeResponse:
    def __init__(self, state, errormsg, answer):
        self.state = state
        self.errormsg = errormsg
        self.answer = answer


def _load_gemini_module():
    sys.path.insert(0, str(SCRIPTS_DIR))

    fake_rospy = types.SimpleNamespace(
        loginfo=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
    )
    sys.modules["rospy"] = fake_rospy

    fake_srv_module = types.ModuleType("seu_llm.srv")
    fake_srv_module.seu_llm = object
    fake_srv_module.seu_llmResponse = _FakeResponse
    sys.modules["seu_llm"] = types.ModuleType("seu_llm")
    sys.modules["seu_llm.srv"] = fake_srv_module

    spec = importlib.util.spec_from_file_location(
        "gemini_llm_service_under_test",
        SERVICE_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GeminiRetryBehaviorTest(unittest.TestCase):
    """约束 Gemini 网络失败时应原样重试，而不是误判为格式错误。"""

    def test_transport_failure_retries_same_prompt(self):
        module = _load_gemini_module()
        module._load_plan_examples = lambda: {
            "Tell me how many boxes there are in the kitchen": (
                "[['move_to', 'kitchen'], ['count_object', 'box', 'kitchen']]"
            )
        }

        prompts = []
        responses = [
            {
                "ok": False,
                "status_code": None,
                "text": "",
                "raw_text": "",
                "data": None,
                "error": "RequestException: SSL EOF",
            },
            {
                "ok": True,
                "status_code": 200,
                "text": "[['move_to', 'kitchen'], ['count_object', 'box', 'kitchen']]",
                "raw_text": "",
                "data": {},
                "error": "",
            },
        ]

        def fake_call(prompt, model):
            prompts.append(prompt)
            return responses.pop(0)

        module.call_gemini_generate_content = fake_call

        req = types.SimpleNamespace(command="Tell me how many boxes there are in the kitchen")
        response = module.analyze(req)

        self.assertTrue(response.state)
        self.assertEqual(response.answer, "[['move_to', 'kitchen'], ['count_object', 'box', 'kitchen']]")
        self.assertEqual(len(prompts), 2)
        self.assertEqual(prompts[0], prompts[1])
        self.assertNotIn(module.PROMPT_ADD_FORMAT, prompts[1])


if __name__ == "__main__":
    unittest.main()
