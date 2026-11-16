# 实测验收记录

日期：2026-11-16。环境：Linux、Python 3.12.3、Node.js 24.19.0、Pinocchio 4.1.0、Coal 3.0.3。

## 自动验证

- 28 项测试通过，包含真实 Chromium 浏览器验收；可选 Xacro 已安装。
- 计算测试覆盖可复现采样、分类、代表姿态、mimic、负倍率限位、Top-K、锁定、统计失效和测速。
- API 测试覆盖后台进程、取消、过期结果、批量修改原子性、工程恢复和模型指纹变化。
- 使用 OCS2 原始 `loadStdVectorOfPair` 头文件及 Boost INFO 解析器读取实际输出，覆盖两类列表、数值、全局开关和空集合；按 OCS2 的几何扩展逻辑验证 Pinocchio 去重。
- 浏览器验收覆盖直接拖动 collision 几何、矩阵高亮与实时距离、采样、Top-K 预览/应用/撤销、测速、逐对距离及 INFO 显式确认；追加验证切换模型不会残留旧关节值，只有 collision 的模型在编辑 mimic 驱动关节后仍与后端变换一致。
- 前端构建通过；Python 导入与未定义符号检查通过；前端依赖安装审计为 0 个已知漏洞。

OCS2 兼容验证针对原生配置读取及几何配对语义，未运行完整 OCS2 MPC 优化器。

## MABI 真实模型

模型：`examples/mabi_mobile/urdf/mabi_mobile.urdf`，名称 `mm_arm`。

- 7 个有 collision 的 link，11 个 object。
- 21 个候选 link 对，展开为 50 个 object 对。
- 种子 42，10,000 次采样，Always 阈值 95%，默认安全距离 0.05 m，全部自动规则开启。
- 分析耗时约 **5.40 s**（后台启动与资源加载时间不计入此数值）。
- 自动规则后保留 **5 个 link 对 / 16 个 object 对**，未额外应用 Top-K。
- 保留对：ARM–FOREARM、ELBOW–arm_base、FOREARM–arm_base、WRIST_1–arm_base、WRIST_2–arm_base。

测速使用独立种子 100045、200 个相同测试位形，分别预热，包含运动学和几何位姿更新，不提前退出。

| 计算类型 | 全量平均 / P95 | 保留集合平均 / P95 | 平均加速比 |
|---|---|---|---|
| 碰撞判断 | 0.376 / 0.420 ms | 0.138 / 0.215 ms | 2.73× |
| 距离计算 | 0.399 / 0.570 ms | 0.134 / 0.156 ms | 2.98× |

这些是本次机器和 Python 调用路径下的实测值，并非其他机器、模型或原生 C++ OCS2 的性能承诺。更密集采样会发现低频碰撞，少量采样与 10,000 次采样的保留集合可能不同。

已在本地 `config/` 生成这次结果的 SRDF、INFO、YAML。生成产物由 Git 忽略，源码、依赖锁文件、测试及文档纳入版本管理。

## RealBot01 加载与可调整布局修复

- 直接加载 `/home/leon/workspace/RealBot01_description/robot.urdf`，不修改源文件：59 个 link、58 个 joint、63 个 collision object、26 个有碰撞几何的 link，对应 325 个 link 对。
- 原错误来自将 fixed 关节的零轴判为无效。fixed 轴不参与运动学，现在跳过该检查；活动关节仍要求有限、非零三维轴，错误包含关节名称。回归测试验证 fixed 零轴不改变几何索引和位姿。
- 原始 RealBot01 在真实浏览器中成功显示，前后端 link 及 collision 原点一致，矩阵包含 650 个对称单元格。
- 浏览器验证左右侧栏宽度、三维/矩阵上下比例、单元格宽度、行名列宽、双击复位、键盘微调、刷新后布局恢复及窄窗口下滚动；画布通过 ResizeObserver 同步尺寸。
- 完整回归结果：28 passed；前端生产构建、Python 静态检查与 Git 空白检查通过。存在一个来自 Starlette TestClient 的 httpx 弃用提示，不影响测试结果。
