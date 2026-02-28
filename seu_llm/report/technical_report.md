# seu_llm Package Technical Report

## 1. 使用方法与功能描述

### 1.1 命令使用

**启动主服务（Baidu ERNIE-Bot 4.0）：**
```bash
rosrun seu_llm baidu_llm_service.py
```

**测试服务调用：**
```bash
# 任务规划
rosservice call /baidu_llm_service "task: 'Go to the kitchen and find an apple' command: 'analyze'"

# 问题回答
rosservice call /baidu_llm_service "task: 'Who is the most handsome person in Canada?' command: 'answer'"
```

**启动讯飞星火服务（备选）：**
```bash
rosrun seu_llm xf_llm_service.py
```

### 1.2 功能详细描述

#### 主要功能
该包提供基于大语言模型（LLM）的机器人任务规划与问答服务，支持：

1. **任务规划（Task Planning）**：将自然语言指令分解为可执行的基元动作序列
2. **问题回答（Question Answering）**：回答用户提出的问题，限制在30个英文单词内

#### 支持的基元动作
- `move_to`: 移动到指定位置
- `look_for_person`: 查找特定人员
- `look_for_obj`: 查找特定物体
- `follow`: 跟随某人
- `lead_to`: 引导某人到指定位置
- `person_analyze`: 分析人员信息
- `info_inform`: 告知信息
- `grasp`: 抓取物体
- `deliver`: 递送物体
- `put_on`: 放置物体
- `answer`: 回答问题

#### 实现模型
- **主推荐**：百度文心一言 ERNIE-Bot 4.0（`baidu_llm_service.py`）
- **备选方案**：
  - 讯飞星火大模型（`xf_llm_service.py`）
  - 文心一言 3.5（`task_planner_wxyy.py`）
  - Llama 2 70B（`task_planner_llama_2_70b.py`）
  - TigerBot（`task_planner_tigerbot.py`）

---

## 2. 外部依赖

### 2.1 ROS依赖的话题（Topics）
本包为服务节点，**不订阅或发布任何话题**。

### 2.2 ROS依赖的服务（Services）

**提供的服务：**
| 服务名 | 服务类型 | 描述 |
|--------|---------|------|
| `/baidu_llm_service` | `seu_llm/seu_llm` | 百度文心一言服务 |
| `/xf_llm_service` | `seu_llm/seu_llm` | 讯飞星火服务（可选） |

**服务定义：**
```
# Request
string task       # "analyze" 或 "answer"
string command    # 具体任务内容或问题
---
# Response
bool state        # 成功/失败
string errormsg   # 错误信息
string answer     # 返回结果
```

### 2.3 外部代码依赖

**Python库依赖：**
- `requests`: HTTP请求库
- `json`: JSON处理
- `re`: 正则表达式
- `rospy`: ROS Python接口
- `websocket`: WebSocket通信（讯飞星火）
- `hashlib`, `hmac`, `base64`: 加密认证

**ROS包依赖：**
- `roscpp`
- `rospy`
- `std_msgs`
- `message_generation`
- `message_runtime`

**外部API依赖：**
1. **百度千帆平台**：`https://qianfan.baidubce.com/v2/chat/completions`
2. **讯飞星火平台**：`ws://spark-api.xf-yun.com/v2.1/chat`
3. **TigerBot API**：`https://api.tigerbot.com/v1/chat/completions`

### 2.4 配置文件依赖

| 文件路径 | 用途 | 格式 |
|---------|------|------|
| `scripts/plan_sets/plan_set_2025.json` | 任务规划示例模板（42个示例） | JSON |
| `scripts/planner/xf/config.json` | 讯飞星火API配置 | JSON |

---

## 3. 可调超参数表

### 3.1 百度文心一言服务超参数

| 参数名 | 位置 | 默认值 | 取值范围 | 描述 |
|--------|------|--------|---------|------|
| `Authorization` | `baidu_llm_service.py:17` | 需配置 | Bearer Token | API密钥 |
| `model` | `baidu_llm_service.py:84,167` | `"ernie-4.0-turbo-8k"` | 模型名称 | 使用的模型版本 |
| `plan_set_path` | `baidu_llm_service.py:12` | 需配置 | 文件路径 | 任务模板路径 |
| `max_retry` | 代码中固定 | `5` | 整数 | 最大重试次数 |
| `answer_word_limit` | `baidu_llm_service.py:81` | `30` | 整数 | 回答字数限制 |

### 3.2 讯飞星火服务超参数

| 参数名 | 位置 | 默认值 | 取值范围 | 描述 |
|--------|------|--------|---------|------|
| `app_id` | `config.json:3` | 需配置 | 字符串 | 应用ID |
| `api_key` | `config.json:5` | 需配置 | 字符串 | API密钥 |
| `api_secret` | `config.json:4` | 需配置 | 字符串 | API密钥 |
| `domain` | `config.json:6` | `"generalv2"` | `"generalv1"`, `"generalv2"` | 模型版本 |
| `temperature` | `config.json:10` | `0.5` | `[0, 1]` | 随机性控制 |
| `top_k` | `config.json:11` | `4` | `[1, 6]` | 候选采样数 |
| `max_tokens` | `config.json:12` | `2048` | `[1, 4096]` | 最大输出tokens |

### 3.3 Prompt工程参数

| 参数名 | 位置 | 默认值 | 描述 |
|--------|------|--------|------|
| `action_base` | `baidu_llm_service.py:131` | 12个基元动作 | 可执行动作集合 |
| `prompt_known` | `baidu_llm_service.py:156-162` | 预设场景知识 | 环境先验知识 |
| `answer_reference` | `baidu_llm_service.py:19` | 50个QA示例 | 问答参考示例 |

---

## 4. 代码详细解析与Review

### 4.1 主服务文件：baidu_llm_service.py

#### 4.1.1 全局配置 (Lines 1-19)

```python
seuhome_llm_name = 'baidu_llm_service'
plan_set_path = "/home/syw/code/robocup_nation/2024code/GPSR/src/seu_llm/scripts/plan_sets/plan_set_2025.json"
Authorization = 'Bearer bce-v3/ALTAK-XtnGWJsMbwfdXWupUdgwo/b5b0b7853ac8bc6c80efc4e77bec4867c30281e5'
```

**功能**：定义服务名称、任务模板路径和API认证token。

**Review**：
- ⚠️ **硬编码路径**：`plan_set_path`应使用相对路径或从ROS参数服务器读取
- ⚠️ **安全风险**：API密钥硬编码在代码中，建议使用环境变量或配置文件
- ✅ **建议**：移至`config.yaml`或使用`rosparam`

#### 4.1.2 主服务函数：llm_service() (Lines 20-73)

```python
def llm_service(req):
    if req.task == 'analyze':
        return analyze(req)
    elif req.task == 'answer':
        return get_answer(req)
    # ... 后续代码为旧版本，存在冗余
```

**功能**：ROS服务回调函数，根据`req.task`类型路由到不同处理函数。

**Review**：
- ✅ **清晰的路由逻辑**
- ⚠️ **代码冗余**：Lines 27-73包含未被调用的旧版实现，应删除
- ⚠️ **缺少异常处理**：如果`req.task`既不是'analyze'也不是'answer'，无返回值

#### 4.1.3 问答函数：get_answer() (Lines 75-122)

```python
def get_answer(req):
    rospy.loginfo("Start to answer the question")
    tts_question = req.command
    url = "https://qianfan.baidubce.com/v2/chat/completions"
    
    answer_prompt = "Please answer the following questions in English in no more than 30 words with format Answer: xxx and the numbers are needed represented by English words."
    answer_prompt += f"\n\nquestion:{tts_question}.Follow are some references:{answer_reference}"
```

**功能**：调用文心一言API回答问题。

**工作流程**：
1. 构建包含50个参考QA的提示词
2. 发送POST请求到百度API
3. 使用正则表达式提取答案：`r'"content":"Answer: (.*?)"'`
4. 失败最多重试5次

**Review**：
- ✅ **Few-shot Learning**：使用50个参考问答提高准确性
- ✅ **格式约束**：要求答案以"Answer:"开头
- ⚠️ **正则表达式问题**：`pattern = r'"content":"Answer: (.*?)"'` 过于严格，如果LLM不完全按格式返回会失败
- ⚠️ **超长prompt**：`answer_reference`有约1万字符，可能超过token限制
- 🔧 **建议**：使用JSON解析代替正则表达式

#### 4.1.4 任务规划函数：analyze() (Lines 126-237)

```python
def analyze(req):
    print("Start to analyze the command")
    url = "https://qianfan.baidubce.com/v2/chat/completions"
    
    # 定义基元动作
    action_base = "move_to, speak, answer, look_for_person, look_for_obj, follow, lead_to, person_analyze, info_inform, grasp, deliver, put_on,answer"
    prompt = f"from actions import {action_base}"
```

**功能**：将自然语言任务分解为基元动作序列。

**工作流程**：
1. **加载示例**：从`plan_set_2025.json`读取42个示例
2. **构建Prompt**：
   - 基元动作列表
   - 42个任务示例
   - 场景先验知识（物品位置）
   - 输出格式要求
3. **LLM调用**：发送到ERNIE-Bot 4.0
4. **结果验证**：
   - 正则匹配：`r"\[\[.*?\]\]"`
   - 检查动作是否在`action_base_lst`中
5. **重试机制**：最多5次，添加提示词

**Review**：
- ✅ **Few-shot Prompting**：42个高质量示例
- ✅ **结果验证**：检查动作合法性
- ✅ **自适应重试**：根据失败原因调整提示词
- ⚠️ **场景知识硬编码**：`prompt_known`写死家庭环境，不灵活
- ⚠️ **动作重复**：`action_base`中`answer`出现两次
- 🔧 **建议**：场景知识从配置文件加载

**关键Prompt（简化）**：
```python
prompt_known = """There are something already known:
    foods on the desk in the bedroom(kind contain chip, biscuit, bread) and on the TV cabinet in the living room;
    drinks on the side table in the living room(kind contain sprite, cola, water);
    cleaning stuffs on the dishwasher or sink in the kitchen;
    Others: dont need count without key how many; must analyze if want to tell somebody; 
            you must grasp obj before deliver; you can direct move to furniture"""
```

#### 4.1.5 主程序入口 (Lines 239-243)

```python
if __name__ == '__main__':
    rospy.init_node("llm_service_node")
    server = rospy.Service(seuhome_llm_name, seu_llm, llm_service)
    rospy.loginfo("Ready to analyze the command")
    rospy.spin()
```

**功能**：初始化ROS节点并注册服务。

**Review**：
- ✅ **标准ROS服务节点结构**
- ✅ **清晰的日志输出**

---

### 4.2 测试文件：test.py

**功能**：独立测试脚本，不依赖ROS环境。

**差异点**：
1. 使用旧版API（`get_access_token()`方法）
2. 返回值为元组而非`seu_llmResponse`对象
3. 包含简单的`Prompt`类用于模拟请求

**Review**：
- ⚠️ **代码不同步**：与主服务逻辑不一致，维护困难
- 🔧 **建议**：统一测试代码，或添加注释说明差异

---

### 4.3 任务模板：plan_set_2025.json

**结构**：42个键值对，键为自然语言任务，值为对应的动作序列。

**示例**：
```json
{
  "Navigate to the waste basket then meet Paris and say your teams affiliation": 
  "[['move_to', 'waste_basket'], ['look_for_person', 'Paris', 'nocount'], ['info_inform', 'team_affiliation']]"
}
```

**任务类型覆盖**：
1. **导航+人员交互**（13个）：找人、跟随、引导
2. **物体操作**（8个）：抓取、放置、递送
3. **信息查询**（12个）：计数、属性查询、姿态识别
4. **社交交互**（6个）：打招呼、介绍、回答问题
5. **复合任务**（3个）：多步骤组合

**Review**：
- ✅ **覆盖全面**：涵盖GPSR竞赛主要任务类型
- ✅ **格式规范**：统一的参数格式（如`'nocount'`, `'none'`）
- ⚠️ **缺少注释**：`plan_set_2025_with_notes.py`提供了中文注释但未在JSON中体现
- 🔧 **建议**：添加任务分类标签，便于检索

---

### 4.4 讯飞星火实现：xf_llm_service.py

#### 核心类：SparkSpeaker (spark_speaker.py)

**功能**：封装讯飞星火WebSocket通信。

**关键方法**：

1. **WsParam.create_url()** (Lines 76-105)
   - 生成带鉴权的WebSocket URL
   - 使用HMAC-SHA256签名

2. **SparkSpeaker.on_message()** (Lines 261-306)
   - 处理流式返回数据
   - 状态：0=首次，1=中间，2=完成

3. **SparkSpeaker.ask()** (Lines 437-450)
   - 单次问答接口
   - 支持流式打印

**Review**：
- ✅ **完善的WebSocket处理**
- ✅ **流式输出支持**
- ✅ **Token计费统计**
- ⚠️ **配置文件路径硬编码**：Line 22
- ⚠️ **日志文件管理**：可能累积大量日志

---

### 4.5 其他LLM实现对比

| 实现 | 模型 | 优点 | 缺点 |
|------|------|------|------|
| `baidu_llm_service.py` | ERNIE-Bot 4.0 | 效果最佳，支持问答和规划 | 需要付费API |
| `xf_llm_service.py` | 讯飞星火 | 免费200万token | WebSocket复杂度高 |
| `task_planner_wxyy.py` | ERNIE-Bot 3.5 | 旧版稳定 | 仅支持规划，无问答 |
| `task_planner_llama_2_70b.py` | Llama 2 70B | 开源 | 需要百度托管，效果一般 |
| `task_planner_tigerbot.py` | TigerBot 70B | API简单 | 需要付费，稳定性未知 |

---

## 5. 关键技术点总结

### 5.1 Prompt工程
- **Few-shot Learning**：使用42个示例教学
- **格式约束**：要求输出`[['action', 'param'], ...]`格式
- **上下文注入**：环境先验知识（家具-物品映射）

### 5.2 容错机制
- **正则验证**：检查输出格式
- **动作合法性检查**：验证动作在预定义集合中
- **重试策略**：最多5次，动态调整提示词

### 5.3 优化建议

**高优先级**：
1. ⚠️ **配置外部化**：将路径、API密钥移至配置文件
2. ⚠️ **异常处理增强**：添加网络超时、JSON解析失败处理
3. ⚠️ **代码去重**：删除旧版实现

**中优先级**：
4. 🔧 **场景知识可配置**：支持多场景切换
5. 🔧 **日志管理**：添加日志轮转
6. 🔧 **单元测试**：添加pytest测试用例

**低优先级**：
7. 💡 **性能优化**：缓存常见任务规划结果
8. 💡 **可视化**：添加任务规划可视化工具

---

## 6. 部署检查清单

- [ ] 修改`baidu_llm_service.py:12`的`plan_set_path`为实际路径
- [ ] 配置`baidu_llm_service.py:17`的`Authorization` token
- [ ] （可选）修改`spark_speaker.py:22`的`config_path`
- [ ] （可选）在`config.json`中配置讯飞星火API密钥
- [ ] 确保已安装`requests`、`websocket-client`库
- [ ] 编译ROS包：`catkin build seu_llm`
- [ ] 测试服务：`rosservice list | grep llm`

---

**报告生成时间**：2025-12-26  
**代码版本**：基于当前seu_llm目录  
**文档作者**：Antigravity AI Assistant
