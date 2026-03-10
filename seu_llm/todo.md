# seu_llm Prompt TODO

本文档记录本轮已经落地的 prompt 改造方向，以及后续建议继续强化的点。重点不是“大模型怎么写得更花”，而是让 LLM 输出和新主状态机/新 look_for_object 四服务接口严格对齐，减少错误动作和中间伪变量。

## 已落地的关键改法

### 1. 从动作集合里移除 `look_for_obj`
- 建议：只保留 `find_object`、`relate_object`、`count_object`、`analyze_object` 四个显式动作。
- 原因：旧 `look_for_obj(target, feature)` 把普通查找、关系查找、计数、分析全塞进一个动作里，LLM 很容易把 `feature` 用乱，也会让 SMACH 继续写大量分发判断。

### 2. 让每个物体查询动作的参数个数固定
- 建议：
  - `['find_object', target_class, search_scope]`
  - `['relate_object', target_class, reference_class, relation, search_scope]`
  - `['count_object', target_class, search_scope]`
  - `['analyze_object', target_class, metric, search_scope]`
- 原因：固定参数位比“靠 feature 猜模式”更稳定，后续你在 LLM 层做解析、做纠错、做 few-shot 示例都更容易约束。

### 3. 物体查询动作前不要再额外生成 `move_to`
- 建议：只保留查询动作本身，把 `search_scope` 直接交给对象查询模块。
- 原因：新的 `seu_look_for_object` 会自己读取 `seu_nav` 的锚点 YAML，在 scope 内巡航、转圈、发现候选、抵近复核。前面再加一个 `move_to same_scope` 只是重复导航。

### 4. 操作类动作统一引用 `current_query_object`
- 建议：当 `grasp`、`deliver`、`put_on` 的目标来自最近一次物体查询时，统一写成：
  - `['grasp', 'current_query_object']`
  - `['deliver', 'current_query_object']`
  - `['put_on', 'current_query_object', place]`
- 原因：关系查询和分析查询的最终物体标签在规划阶段往往未知。继续让 LLM 瞎写 `obj_name` 或重复猜标签，容易和真实识别结果脱节。

### 5. 播报类动作统一引用 `query_result_label` / `query_result_count`
- 建议：
  - 讲“找到的是谁”时用 `['info_inform', 'query_result_label']`
  - 讲“数了多少个”时用 `['info_inform', 'query_result_count']`
- 原因：旧的 `obj_name`、`obj_count` 本质上是伪变量，占位字符串和 ROS Param 旁路耦合太深。新状态机已经改成直接读最近一次查询结果，不该再让 LLM 生成旧占位名。

## 还建议继续加强的 prompt 规则

### 6. 明确要求地点和类别优先使用下划线命名
- 建议：在 prompt 里强调 `orange_juice`、`cleaning_supply`、`storage_table`、`living_room` 这类形式优先。
- 原因：下划线 token 更适合做动作参数，也更适合后续 LLM 规范化、规则检查和关键词映射，减少 `"storage table"` / `"storage_table"` 混用。

### 7. 明确禁止无意义的中间伪变量
- 建议：在 prompt 里明确写：
  - 不要输出 `obj_name`
  - 不要输出 `obj_count`
  - 不要输出 `found_object`
  - 不要输出 `target_obj`
- 原因：这些词对 LLM 来说很“像代码变量”，但对当前动作系统来说并不是可执行数据通道。真正的数据通道应该是 `current_query_object`、`query_result_label`、`query_result_count`。

### 8. 要求 `count_object` / `analyze_object` 后默认不要接 `grasp`
- 建议：在 prompt 中明确：
  - `count_object` 用于统计，不用于抓取
  - `analyze_object` 用于排序/比较/回答，不用于抓取
  - 只有 `find_object` 或 `relate_object` 的结果才应该直接进入抓取链
- 原因：这样能减少“先分析最大物体再直接抓取”的歧义，也更符合你当前设计的执行边界。

### 9. 明确 `search_scope` 必须是可被导航层解析的普通地点名
- 建议：prompt 中加入一句：如果用户说的是家具、区域别名或自然语言描述，先在 LLM 层归一化成 `seu_nav` 可识别的地点/锚点名再输出。
- 原因：你已经明确说过，特殊 scope 会在 LLM 层转换；那就要把这条规则写死，不要把不规范字符串继续往下游传。

### 10. 明确 few-shot 样例中要覆盖四类物体查询
- 建议：样例至少覆盖：
  - 普通查找并抓取
  - 关系查找并抓取
  - 计数并播报
  - 分析并播报
- 原因：现在 few-shot 对 LLM 的动作习惯影响很大。如果样例里仍大量保留旧 `look_for_obj`，即使主 prompt 改了，模型也容易回退到旧写法。

### 11. 最好再加一条“禁止重复生成等价动作”
- 建议：比如明确禁止：
  - `['move_to', 'kitchen']` 后紧跟 `['find_object', 'apple', 'kitchen']`
  - 同一任务连续两个对象查询动作使用同一个 scope 但没有中间目的
- 原因：这类重复步骤在语言上看似合理，但对实际机器人执行是纯浪费时间。

## 对应的主状态机配套约束

为了让上述 prompt 规则真正有意义，主状态机这边已经按同一方向改造：

- `gpsr_smach.py` 已用四个显式状态替换旧 `LookForObjState`
- `grasp` 改为读取 `object_query_result.selected_object`
- `info_inform` 对对象标签/数量改为直接读取最近一次查询结果
- `look_for_obj` 旧动作不再是主路径

## 后续建议

如果你下一步继续改 prompt，我建议优先顺序是：

1. 先把 system prompt 和 few-shot 样例统一成四个新动作。
2. 再加一层动作参数检查，重点卡参数个数和禁止旧 token。
3. 最后再做地点名规范化和更多长尾规则。

这样改动顺序最稳，因为第一步先解决“动作名错”，第二步解决“参数结构错”，第三步才是“语义更精细”。
