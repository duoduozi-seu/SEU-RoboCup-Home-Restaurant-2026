import json

import requests

# 明文配置（不读取环境变量）
API_KEY = "sk-ZzVTvgKk64GDcEFq0vf2RJzbReb6orUR9mHB0ufhwR8F2UPE"
BASE_URL = "https://api.viviai.cc"
MODEL = "gemini-2.5-flash"
PROMPT = "Hi"

# apifox 文档接口：POST /v1beta/models/{model}:generateContent
url = f"{BASE_URL.rstrip('/')}/v1beta/models/{MODEL}:generateContent"

payload = {
    "contents": [
        {
            "role": "user",
            "parts": [
                {
                    "text": PROMPT
                }
            ]
        }
    ]
}

headers = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json",
}

response = requests.post(
    url,
    headers=headers,
    data=json.dumps(payload),
    timeout=60,
)

print(f"HTTP {response.status_code}")
print(response.text)

# 额外打印解析后的文本和 token 使用量，便于测试
try:
    data = response.json()
    candidates = data.get("candidates", [])
    if candidates:
        parts = candidates[0].get("content", {}).get("parts", [])
        answer_text = "".join(part.get("text", "") for part in parts if isinstance(part, dict))
        if answer_text:
            print("\n---- Parsed text ----")
            print(answer_text)
    usage = data.get("usageMetadata", {})
    if usage:
        print("\n---- Usage ----")
        print(
            "promptTokenCount:",
            usage.get("promptTokenCount", 0),
            "candidatesTokenCount:",
            usage.get("candidatesTokenCount", 0),
            "totalTokenCount:",
            usage.get("totalTokenCount", 0),
        )
except ValueError:
    pass
