# LLM Service Package

这是一个ROS功能包，用于实现大语言任务规划与问答功能的封装，采用ERNIE-Bot-4（文心一言4.0）大语言模型。

## Installation

为了能够正常使用文心一言4.0大语言模型，请在以下网站申请测试资格：
    https://cloud.baidu.com/survey/qianfaneb4.html
通常会在填写表单申请后一周内收到相关短信通知

## Usage

要启动该服务，运行如下命令：

    rosrun seu_llm baidu_llm_service.py

要在终端中测试该服务的调用，请运行：

    rosservice call /baidu_llm_service "task: 'command or question' command: 'analyze or answer'" 

## Node llm_service_node

这是一个应用大语言模型进行指令解析或者问题回答的节点，它接受用户的任务类型（analyze/answer）和任务内容，根据任务类型向大模型发送不同的prompt，返回大模型的解答内容和错误码。
目前应用的是文心一言4.0模型，可以实现指令解析和问题回答。另外在planner文件夹里还有几种其他的大模型，可以实现指令解析，但效果不如文心一言4.0理想。如果想要换成别的模型，可以仿照baidu_llm_service.py将代码封装成rosservice并添加回答问题的部分。
路径修改：需要修改json文件的全局路径

