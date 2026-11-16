# URDF 自碰撞矩阵生成与碰撞对筛选工具

## 1. 总体方案

采用 robot_viewer 网页界面组件与本地 Python 服务，首版面向 Linux，不依赖 ROS。

- 复用 robot_viewer 的 URDF 适配器、统一模型和关节拖动组件，保留许可证；实现专用的碰撞配置界面。
- Pinocchio + Coal 负责运动学、碰撞检测与距离计算，固定依赖版本。
- FastAPI 提供本地接口，后台进程执行采样与测速，WebSocket 推送进度，支持取消。
- 流程：加载模型 → 排除 link → 采样分类 → 矩阵审查 → Top-K → 测速 → 导出。
- 首版不包含环境碰撞、闭链求解、末端拖动 IK。

## 2. 模型加载与界面

### 模型加载

- 支持本地 URDF、模型目录；选装 xacro 后支持展开及参数输入。
- 支持相对路径、绝对路径及用户配置的 package:// 路径映射。
- 支持多 collision 几何，fixed、revolute、continuous、prismatic 和 mimic 关节。
- 缺失资源、无效几何、前后端模型不一致和不支持的关节类型，明确提示并阻止分析。
- fixed 关节的 axis 不参与运动学，允许常见导出器生成的零轴；仅对活动关节要求有限、非零的三维运动轴，错误信息包含关节名称。以 RealBot01 的原始 URDF 验证，无需修改源模型。
- 后端建立统一 link、joint、collision object 映射；前端按显式映射显示，不推测浏览器加载顺序。
- 排除 link 不删除运动学链、显示或关节控制。

### 布局与交互

- 左侧：模型、包路径、link 列表、参与开关与搜索。
- 中间：三维视图，支持 collision/visual 显隐、透明度、相机旋转、缩放和聚焦。
- 右侧：采样、筛选、选中对详情、关节滑条与数值输入、范围与锁定、测速。
- 下方：可调整高度的二维矩阵，带分类图例、过滤与批量操作。
- 布局使用明确的拖动分隔条：左右调整侧栏和中央工作区宽度，上下调整三维视图与矩阵的高度比例；支持键盘微调、双击恢复默认值，并记住本机布局。
- 矩阵单元格保持紧凑，不强制拉伸填满工作区；支持调整单元格宽度和行名列宽，超出空间时保留双向滚动及固定表头。调整区域尺寸后自动更新三维画布与相机比例。
- 支持直接拖动连杆调整关节，面板同步；mimic 自动跟随，实时更新选中对距离。
- 鼠标悬停在可见的 collision 或 visual 部件上时，在指针旁自动浮窗显示所属 URDF link 名称，包括固定部件；浮窗跟随鼠标并限制在三维视图区内，不拦截点击或拖动。移出部件/视图区、开始拖动、切换显示层或模型时隐藏，隐藏几何不触发提示。
- 左键单击三维视图空白处时，取消碰撞对选中与矩阵选中边框，恢复全部部件的正常颜色与透明度，清空选中对详情；不改变机器人位形、矩阵勾选、锁定及导出集合。点击部件、工具栏或矩阵，以及拖动空白处旋转/平移视角均不触发取消。

### 矩阵

- 对称 link × link 矩阵，对角线不可编辑；勾选表示禁用检测。
- 选中单元格高亮两个 link，复选框修改选择，两侧对称位置同步。
- 显示禁用原因、碰撞率、接近率、最小距离、object 对数量及索引映射。
- 两个选中 link 用不同颜色显示，其他模型淡化。
- 提供碰撞姿态、最近姿态回放和恢复原姿态；普通选中不改变位形。
- 支持名称/分类过滤、批量启用/禁用、必须保留/排除锁定、恢复自动建议。
- 每个 link 对可设置安全距离覆盖值，显示继承后的有效值。

## 3. 分析、筛选与测速

### 默认值与采样

| 参数 | 默认值 |
|---|---|
| 样本数 | 10000 |
| 随机种子 | 42，保存到工程 |
| Always 阈值 | 95% |
| 全局安全距离 | 0.05 m |
| continuous 范围 | [-π, π] |
| 其他关节范围 | URDF 限位，可缩小 |
| 默认姿态 | 零位，越界值裁剪到限位 |

- 无有效范围的活动关节需要填写范围或锁定，mimic 不独立采样。
- 固定根部整体刚体运动，不采样全局位姿。
- 不排除碰撞样本；将驱动关节范围与 mimic 派生的可行区间相交。
- link 对展开成两端所有 object 的交叉组合；任一组合碰撞即判为碰撞，距离取最小值。
- 保存碰撞次数、接近次数、最小距离和代表姿态；接近包含碰撞，不按 mesh 穿透深度排名。
- 接近阈值优先使用逐对覆盖值，否则继承全局默认值。

### 自动禁用

按 Adjacent → Default → Always → Never 顺序确定主要自动禁用原因，同时保留全部分类信息，各规则可独立关闭。

- Adjacent：相邻 link，包括中间只有无 collision link 的情况。
- Default：默认姿态碰撞。
- Always：采样碰撞比例达到阈值。
- Never：采样未发生实际碰撞，即使曾接近也禁用；可人工恢复。
- 其他原因：User、Excluded、TopK。

### Top-K

- K 按 link 对计数，同时报告 object 对数量。
- 从当前启用集合按接近率降序、碰撞率降序、最小距离升序、link 名称排序。
- 锁定保留项先占名额；超过 K 时仍全部保留，提示实际数量。
- 先预览再应用，支持撤销。
- 修改范围、样本参数或安全距离后标记统计待更新，阻止使用旧统计筛选。
- 展示数量变化和采样碰撞事件保留比例；采样不能证明整个运动空间安全。

### 测速

- 显示矩阵生成总耗时。
- 使用同一批独立种子的测试位形，对比参与 link 间全部对与最终启用对。
- 碰撞判断和距离计算分别测量，遍历全部 object 对、不提前退出。
- 预热后显示平均、P95、加速比和 link/object 对数量。
- 包含运动学和几何位姿更新，排除网络和渲染；不与采样并行。

## 4. 配置、工程与接口

### 输出文件

默认输出到 config/：

- disabled_collisions.srdf：最终全部禁用 link 对及原因，包含自动规则、link 排除、人工和 Top-K。
- self_collision.info：完全采用 OCS2 selfCollision 结构，同时包含有效 collisionLinkPairs 和 collisionObjectPairs。
- self_collision.yaml：全局开关、默认安全距离、最终启用 link 对及其 object 映射。**不再输出 XML。**

### YAML 结构

```yaml
version: 1
metadata:
  fingerprint: model-sha256
  # 同时附带依赖版本、完整 object 索引映射
self_collision:
  enabled: true
  default_min_distance: 0.05
  collisionLinkPairs:
    - link1: base_link
      link2: arm_link
      enabled: true
      # 不设置 min_distance 时继承全局值
      collisionObjectPairs:
        - object1: 0
          object2: 3
          object1_name: base_link_0
          object2_name: arm_link_0
    - link1: arm_link
      link2: wrist_link
      enabled: true
      min_distance: 0.08
      collisionObjectPairs:
        - object1: 3
          object2: 7
          object1_name: arm_link_0
          object2_name: wrist_link_0
```

- 全局 enabled 默认 true；default_min_distance 默认 0.05，单位米。
- 每个 link 对包含 enabled 和可选 min_distance；object 对继承所属 link 对设置。
- 全局与局部 enabled 均为 true 才有效，距离必须有限且非负。
- 只导出最终启用集合，生成时逐对 enabled 均为 true，允许消费端后续修改。
- 全局 enabled 不清空集合，也不改变 SRDF。

### OCS2 INFO 完全兼容要求

- 使用 Boost Property Tree INFO 语法及原字段名，不添加自定义有效字段。
- 包含 activate、collisionObjectPairs、collisionLinkPairs、minimumDistance、mu、delta。
- 列表采用连续的 [0]、[1] 编号，值为带引号的 "link1, link2" 或 "0, 3"。
- activate 映射全局 enabled，minimumDistance 映射全局默认距离，mu/delta 默认 0.01/0.001，可编辑。
- 只输出 selfCollision 配置块，替换/合并到 task.info，不声称它是完整 MPC task 配置。
- 两种配对列表同时有效，验证按 OCS2 扩展逻辑加入 Pinocchio 后正确去重。
- OCS2 原格式不支持逐对距离；存在有效覆盖时，导出前必须提示并由用户明确确认 INFO 仅使用全局距离，YAML 保留覆盖。
- 使用原 OCS2 loadStdVectorOfPair 头文件及 Boost C++ 解析器验证兼容性。

### 索引、工程与接口

- 使用完整 Pinocchio collision 模型的零基 object 索引，排除 link 不重新编号。
- 几何名称、所属 link、模型指纹及计算库版本随导出记录；模型/构建方式变化后需要重新校验索引。
- SRDF 禁用集合与 INFO/YAML 配对集合在不同且有 collision 的 link 对范围内互补，不生成同 link 内部对。
- 工程保存模型引用、资源映射、采样设置、逐对距离、统计、代表姿态、人工修改、筛选、测速和当前位形。
- 恢复时校验模型及版本，变化则使旧统计、选择和 object 索引失效，要求重新分析。
- 工程不复制原模型资源。
- HTTP 提供加载、设置、选择、位形、工程和导出接口，WebSocket 推送后台任务状态。
- 后端维护唯一选择状态；任务绑定模型与设置版本，过期结果不得覆盖新状态。
- 服务只监听 localhost。

## 5. 实施与验收

1. 模型加载、collision 显示、link 排除、关节面板和拖动。
2. 采样分类、矩阵编辑、高亮、姿态回放与取消。
3. 距离覆盖、Top-K、锁定、撤销与测速。
4. 三种配置导出、工程恢复、安装文档、Git 管理与测试。

验收覆盖：全部分类；多 collision/fixed/mimic；相同种子可复现；对称编辑和人工覆盖；锁定超过 K；参数变化失效；取消及过期任务；YAML 开关/继承/覆盖/非法值；INFO 原生读取与去重；三种导出配对互补；前后端运动学和 collision 原点一致；工程恢复及模型变化失效；开放许可证测试模型端到端演示及实测性能。
