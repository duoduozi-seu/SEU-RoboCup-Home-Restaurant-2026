## 模块介绍（共16个，2个无动作基元）
  - 1.导航模块
        代码包:seu_nav
        动作基元:move_to
        状态机类:MoveToState

  - 2.语音播报
        代码包:无（调库pyttsx3）
        动作基元:speak
        状态机类:SpeechState

  - 3.问题回答模块
        代码包:seu_llm
        动作基元:answer
        状态机类:AnswerState
        <!-- 合并至info_inform里 -->

  - 4.大模型任务解析（有扩展）
        代码包:seu_llm
        动作基元:无
        状态机类:AnalyzeState
        扩展：对不在动作列表的动作进行后处理、筛选参数是否在选择范围内

  - 5.主状态机模块
        代码包:seu_smach_main
        动作基元:无
        状态机类:无

  - 6.找人模块（需修改）
        代码包:seu_look_for_person
        动作基元:look_for_person
        状态机类:LookForPersonState
        输入参数 [人物特征，是否计数]
        人物特征: name,waving ,raising_left, raising_right,pointing_left, pointing_right，sitting，standing，lying，什么颜色的什么衣服
        是否计数： count ,nocount
        eg  ['look_for_people, 'raising_right','count']
        输出参数 [state,errormsg,person_count]

  - 7.找物模块（需修改）
        代码包:seu_look_for_obj
        动作基元:look_for_obj
        状态机类:LookForObjState
        输入参数 [物品名称]
        eg ['look_for_obj','fruit']
        输出参数 [state,errormsg]

  - 8.跟随模块
        代码包:seu_follow
        动作基元:follow
        状态机类:FollowState
        输入参赛 [地点]
        地点:地点名、none
        eg  ['follow', 'living_room']
        输出参数 [state,errormsg]
        扩展：多个人时，实时判断有没有跟对人。优化停止逻辑。

  - 9.引导模块（有扩展）
        代码包:seu_lead
        动作基元:lead_to
        状态机类:LeadToState
        输入参赛 [地点]
        地点:地点名
        eg  ['lead_to', 'living_room']
        输出参数 [state,errormsg]
        扩展：实时检测引导的人是否跟上

  - 10.物品特征识别模块（需新增）
        代码包:seu_obj_analyze
        动作基元:obj_analyze
        状态机类:ObjAnalyzeState
        输入参赛 [物品名称，分析类别]
        分析类别: biggest, largest, smallest, heaviest, lightest, thinnest
        eg ['obj_anlyze', 'cleaning_supply', 'biggest']
        输出参数 [state,errormsg,obj_name]

  - 11.物品计数模块（需新增）
        代码包:seu_obj_count
        动作基元:obj_count
        状态机类:ObjCountState
        输入参数 [物品名称]
        输出参数 [state,errormsg,人物特征]

  - 12.人物特征识别模块（需新增）
        代码包:seu_person_analyze
        动作基元:person_analyze
        状态机类:PersonAnalyzeState
        输入参数 [人物特征]
        人物特征: pose, gesture, name, clothe(扩展)
        eg  ['person_analyze' 'gesture']
        输出参数 [state,errormsg,人物特征]

  - 13.信息告知模块（需新增）
        代码包:seu_info_inform
        动作基元:info_inform
        状态机类:MoveToState
        输入参数 [信息类别,原始命令(第一步即保存)]
        信息类别: person_count，gesture, obj_name，obj_count, answer,today_date,current_time,team_name,team_country,team_affiliation,day_of_month,day_of_week,greet,salute, introduce
        eg ['info_inform','person_count']
        输出参数 [state,errormsg]

  - 14.物品抓取模块（需新增）
        代码包:seu_grasp
        动作基元:grasp
        状态机类:GraspState
        输入参数 [物品名称]
        eg ['grasp','fruit']
        输出参数 [state,errormsg]

  - 15.物品传递模块（需新增）
        代码包:seu_deliver
        动作基元:deliver
        状态机类:DeliverState
        输入参数 [物品名称]
        eg ['deliver','fruit']
        输出参数 [state,errormsg]

  - 16.物品放置模块（需新增）
        代码包:seu_put_on
        动作基元:put_on
        状态机类:PutOnState
        输入参数 [物品名称,家具名称]
        eg ['put','fruit','sink']
        输出参数 [state,errormsg]

## 优化项
1.接受命令后记录me的特征，以防初始点多个人时无法区分谁是me