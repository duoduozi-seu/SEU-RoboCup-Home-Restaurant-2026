rosrun seu_info_inform  info_inform_service.py
rosservice call /info_inform_service "team_name" "what is your team name"


  目前我想把所有的干了一些事情之后的二次语音交互（info_inform），比如找到某人说什么、
  看到什么回来汇报、数完东西回来汇报有几个、找到某某给他打招呼\介绍自己\回答问题（先直接播报一句你要干嘛然后语音识别然后llm回答）、等等功能，详细所有可能性见/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src/seu_llm/scripts/plan_sets/plan_set_2025_with_notes.py，都放info_inform里给llm自己自动干，自己组织语言。把最开始的command+组织一下prompt比如我已经干完了前面的事情，现在要你完成info_inform环节+上一个状态机环节获得的信息（就是输入的3个变量+输入变量的背景情况）
  等等，一股脑都丢给llm，并附上实际的例子，让他学会需要输出什么、并规定llm强制的输出格式，方便后面语音播报出去。我要求所有llm包括api等东西都不
  要调用seu_llm节点，全部重新写，用到相关的可以直接复制粘贴并且尽量复制粘贴，方便我理解，但是意思就是不要调用seu_llm的服务，否则prompt很容易混
  在一起。进入info_inform,可以先判断一下是不是有特殊的动作比如：answer（需要先问他要问啥），其他的什么 ['info_inform', 'person_count']、
  ['info_inform', 'query_result_label']等类似直接汇报的，都直接把信息丢给llm然后llm返回啥就直接语音播报啥。这样没有那么多约定的动作，把播报内容
  的理解都交给llm感觉会更智能一点，更不容易出错。

  所以综上：你只需要修改info_inform:
  在该节点抄一些需要的seu_llm的函数过来，而不是直接调用seu_llm，如果能照抄的直接照抄，包括api token这些。
  （1）把所有二次交互从主状态机放回info_inform；
  （2）需要你确认如何把上一个状态机输出的状态和信息传进去（3个变量已经传递进去了）
  （3）完成流程：判断特殊的行为如answer：先播报固定英语语句：意思大概是”你要问什么问题”->拼接prompt->发给llm->播报回答；或者其他情况是直接把已知信息+预先写好的prompt拼接起来->丢给llm->播报回来的消息。
  （4）在info_inform完成供llm学习的任务例子prompt：info_inform环节+上一个状态机环节获得的信息等等->应该输出的东西。比如学会知道任务和现在做完前面的任务、知道他负责的状态是info_inform、前面已经识别到数到了5个apple，那么就应该输出播报有5个apple类似内容。具体所有可能性看/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src/seu_llm/scripts/plan_sets/plan_set_2025_with_notes.py。这个长prompt直接硬编码在scripts另外开的一个文件中，到时候随前面的prompt一并合并后丢给llm。
  （5）规定待拼接的prompt：除了学习的任务例子，还有answer的示例和关于一些team和机器人（自己）信息的常识，具体可以在seu_llm的prompt中找到。这个也需要你看完seu_llm的例子之后，在info_inform中硬编码一份
  （6）在prompt中规定一个简单的输出格式，然后等待输出并解析，用语音说出去。