# 第二批 Codex Task

## 使用方式

本文件是 Task 08–10 的待执行任务说明，不是完成记录。每次只执行用户指定的一项；以下任务当前全部为“未开始”。以项目根目录为工作目录，遵守根目录 [AGENTS.md](../AGENTS.md)，并保持第一批 [Task 01–07](CODEX_TASKS_BATCH_01.md) 的公共契约和回归测试。

| 任务 | 内容 | 前置任务 |
|---|---|---|
| Task 08 | SORT 图片序列可视化与现场数据基线 | 07 |
| Task 09 | Observation History 与 OCM 关联 | 03、04、06 |
| Task 10 | OCR、ORU 与完整 OCSortTracker | 08、09 |

推荐按编号执行。第二批只完成固定步长、纯运动、IoU 关联的 OC-SORT；不引入 ReID、ByteTrack 低置信度二次匹配、动态 dt、时间化超时、Qt 或平台适配。

每项交付均包括：实现和对应测试、必要文档更新、实际构建/测试命令与结果、未验证事项。不得以人工观看代替可自动断言的算法测试，也不得把无身份真值的现场样本描述为 ID Switch 指标已通过。

## 规划依据与当前数据

第一批已经形成 `SortTracker`、固定 CSV Replay 和完整生命周期。原实施计划 Task 11 的 Tentative / Confirmed / Lost / Removed 语义已在第一批 Task 06 落实，第二批不重复开发，只要求新 Tracker 复用同一契约。

规划时本地可见两组现场数据：

| 目录 | 帧数 | 图片 | 原始检测行 | 默认阈值 0.5 可用检测 | 已知限制 |
|---|---:|---:|---:|---:|---|
| `testdata/capture_20260917_单人/` | 275 | 275 | 196 | 172 | 无身份真值；时间步长 127–7296 ms |
| `testdata/capture_20260917_双人/` | 279 | 279 | 228 | 210 | 无身份真值；时间步长 127–3584 ms；仅 4 帧含两条原始检测 |

两组数据都使用现有 `frames.csv`、`detections.csv` 格式，图片为 `images/<六位 frame_id>.jpg`，分辨率为 540×1320。数据中存在漏检、低置信度检测和非均匀时间间隔，适合可视化与冒烟重放，但目前不足以自动证明双人交叉、遮挡恢复或 ID Switch 数量。

现场原始数据视为只读输入。任务执行不得改写图片或 CSV，不得在未确认隐私、授权和仓库体积策略前擅自提交原始数据；系统生成的 `.DS_Store` 不属于测试数据。

## 算法参考基线

OC-SORT 语义以以下一手资料为依据：

- [CVPR 2023 论文](https://openaccess.thecvf.com/content/CVPR2023/html/Cao_Observation-Centric_SORT_Rethinking_SORT_for_Robust_Multi-Object_Tracking_CVPR_2023_paper.html)
- [官方仓库固定提交 `8462e7e729a93ccd3bd995c0a79a890336cb3a0b`](https://github.com/noahcao/OC_SORT/tree/8462e7e729a93ccd3bd995c0a79a890336cb3a0b)
- [官方 MIT License](https://github.com/noahcao/OC_SORT/blob/8462e7e729a93ccd3bd995c0a79a890336cb3a0b/LICENSE)

实现前应在项目文档中记录“论文语义 → 当前 C++ 类型”的对应关系。若翻译或复用官方代码的实质性部分，须保留其许可证和必要声明；不得引入官方仓库中的检测器、Python 运行时、OpenCV 核心依赖或其他无关组件。

## Task 08：SORT 图片序列可视化与现场数据基线

**目标：** 使用现场图片、检测 CSV 和现有 `SortTracker` 逐帧查看检测、轨迹状态及历史，先建立可人工定位问题的基线闭环。

**交付：** `tracker_visualizer`、必要的工具层 CSV 复用代码、`docs/visualizer.md`，以及一份不含身份指标推断的现场 SORT 基线记录。

**执行说明：**

1. OpenCV 只允许作为 `tracker_visualizer` 的可选依赖。增加明确的 CMake 选项，默认关闭；关闭时不得查找 OpenCV，`tracking_core`、`tracker_replay` 和测试的依赖保持不变。
2. 工具读取 `frames.csv`、`detections.csv` 和图片目录。图片按 `frame_id` 映射到六位十进制 `.jpg` 文件；缺图、无法解码、尺寸不一致或 CSV 错误须报告具体帧并非零退出。
3. 复用 Task 07 的 CSV 语义。允许把解析逻辑从 `tracker_replay.cpp` 提取到工具层，但不得放入 `tracking_core`，不得改变已有 Replay 的输入校验、输出格式或默认行为。
4. 每帧恰好调用一次 `SortTracker::update()`。绿色显示 Detection，蓝色显示 Confirmed，黄色显示 Lost，Tentative 使用可区分颜色；标出 track ID，并在可视化层按 ID 保存最近 24 个中心点绘制轨迹。该历史仅用于显示，不进入核心算法。
5. 支持 `Space` 暂停/继续、右方向键单步、`R` 从首帧重置并重新重放、`D` 显示/隐藏 Detection、`T` 显示/隐藏 Track、`H` 显示/隐藏轨迹历史、`Esc` 退出。禁止为此引入 Qt。
6. 对两组现场数据各完成一次重放，记录配置、总帧数、程序退出状态、创建的轨迹 ID 数量、Lost/恢复事件及人工观察到的异常帧。没有身份真值时只写“观察结果”，不写 ID Switch 率或准确率。

**验收：**

- `BUILD_TRACKER_VISUALIZER=OFF` 时，在无 OpenCV 条件下完整配置、构建和 CTest 均通过，且 CMake 缓存中不出现必需的 OpenCV 查找。
- 本机具备 OpenCV 时，开启选项后成功构建；使用小型合成图片序列验证帧映射、颜色/文字、快捷键、重置和结束行为；错误图片路径与缺图场景非零退出。
- 现有 `tracker_replay` 成功/失败集成测试逐项保持通过，同一合成输入输出仍逐字节一致。
- 两组现场数据均能从首帧运行到末帧；实际人工检查的帧和观察结论写入基线记录。若本机没有 OpenCV，明确报告该项未验证，不自动安装系统软件，也不宣称 Task 08 完成。

**范围：** 不实现 OCM、OCR、ORU，不修改跟踪参数，不将人工观察写成自动回归结论，不提交现场原始数据。

## Task 09：Observation History 与 OCM 关联

**目标：** 实现可独立验证的 Observation-Centric Momentum（OCM）关联模块，但暂不创建名称可能误导的半成品 `OCSortTracker`。

**交付：** 内部 observation history / OCM association 头源文件、`tests/test_ocsort_association.cpp` 和 `docs/ocsort_design.md`。现有 `SortTracker` 的行为必须保持不变。

**执行说明：**

1. 每条候选轨迹只记录真实匹配 Detection 的 observation 及对应轨迹年龄；Kalman 预测框和漏检占位不得冒充 observation。记录最近 observation，并能查询当前年龄之前至多 `deltaT` 帧的最近可用 observation。
2. 按官方基线使用帧数窗口 `deltaT=3`。若目标位置发生有效位移，以两个真实 observation 的中心计算单位运动方向；历史不足或零位移时方向项为中性，不产生 NaN 或伪方向。
3. 对每个“轨迹—检测”候选计算从历史 observation 指向当前 Detection 的单位方向，点积裁剪到 `[-1,1]` 后计算：`angleScore = (pi/2 - abs(acos(dot))) / pi`。OCM 相似度为 `IoU + inertia * detection.confidence * angleScore`，初始 `inertia=0.2`。
4. IoU 门限仍决定边是否有效：方向项不能让 `IoU < iouThreshold` 的边变成有效匹配。对有效边最大化 OCM 相似度，并继续使用 Task 04 的确定性 Hungarian 求解；不得复制“先无约束分配再删除”的已修复缺陷。
5. 先在 `docs/ocsort_design.md` 写清坐标顺序、方向定义、历史回退、分数范围和官方参考映射，再实现代码。此任务的参数作为内部函数输入；只有 Task 10 的公共 Tracker 实际消费它们时才加入 `TrackerConfig`。

**验收：**

- observation history 覆盖连续命中、跨漏检、少于 `deltaT`、恰好 `deltaT`、超过 `deltaT`、零位移和 reset/新轨迹隔离。
- OCM 使用可手算案例断言同向、垂直、反向分别产生正、零、负方向项，并验证置信度和 inertia 权重。
- 构造 IoU 接近但方向相反的二人交叉矩阵，证明 OCM 选择方向一致的匹配；同时验证无历史或 `inertia=0` 时退化为现有 IoU 关联。
- 覆盖空矩阵、无效边、非有限输入和等价最优解；所有输出索引完整且无重复。
- 完整 CTest 通过，原有 `SortTracker` 与 Replay 黄金输出不变。

**范围：** 不实现二次关联和 Kalman 重更新，不添加公共 `OCSortTracker`，不使用现场数据代替数值单元测试，不调参。

## Task 10：OCR、ORU 与完整 OCSortTracker

**目标：** 在固定步长 SORT 生命周期上组合 OCM、Observation-Centric Recovery（OCR）与 Observation-Centric Re-Update（ORU），提供独立、可对照的 `OCSortTracker`。

**交付：** `include/tracking/ocsort_tracker.h`、对应内部实现与测试；扩展 `TrackerConfig`、Replay 和可视化工具的 tracker 选择；完成 `docs/ocsort_design.md` 与公共契约说明。

**执行说明：**

1. `OCSortTracker` 实现 `ITracker`，公共头保持无 Eigen、OpenCV 和 Qt 类型。保留 `SortTracker` 作为基础算法基线，不在原类中用隐藏开关改变语义。
2. 将 `deltaT=3` 与 `inertia=0.2` 加入 `TrackerConfig`，分别要求 `deltaT>=1`、inertia 为有限 `[0,1]`；构造时验证。两个参数必须被 `OCSortTracker` 实际消费，`SortTracker` 可忽略新增字段但仍验证公共配置。
3. 第一轮关联使用 Task 09 的 OCM 相似度和预测框 IoU 门限。对第一轮双方未匹配项执行 OCR：使用轨迹最后一个真实 observation 与 Detection 的 IoU，以同一 `iouThreshold` 建立有效边并再次进行最大匹配数、最小代价的确定性分配。任何轨迹或 Detection 最多匹配一次。
4. 每次真实 observation 更新后保留该时刻的 Kalman posterior；预测产生的丢失状态不得覆盖这个恢复锚点。轨迹被任一关联阶段重新激活时执行 ORU：恢复最后一次真实 observation 后的 posterior，在该 observation 与当前 Detection 之间按跨越帧数线性插值中心、宽和高，并逐帧 predict/update 重放到当前帧。当前 Detection 是虚拟轨迹的终点且只更新一次。全过程保持框有效、状态有限且不直接求逆。
5. 生命周期、ID、confidence、age、lostFrames、时间戳异常的强保证和 reset 行为与 `SortTracker` 契约一致。此任务仍使用固定 `dt=1`；真实 timestamp 只校验顺序，不参与滤波或删除。
6. `tracker_replay` 增加 `--tracker sort|ocsort`，省略时保持当前 `sort` 行为和输出兼容；`tracker_visualizer` 同步支持相同选择。不得增加配置文件系统或 ByteTrack 的低置信度通道。
7. 对现场样本生成 SORT 与 OC-SORT 两份临时输出并记录可复现命令和观察差异。原始数据与生成结果默认不提交；没有身份标注时不计算 ID Switch。

**验收：**

- OCR 测试构造“预测框因漂移低于门限、最后 observation 仍可匹配”的序列，确认恢复原 ID，且无可行 last-observation 边时不会强制匹配。
- ORU 使用可独立计算的短缺失序列验证虚拟 observation 数量、插值坐标、重放后的有限状态及下一帧预测；连续观测时不得触发重放。
- 合成双人交叉/短时遮挡序列中，OC-SORT 保持预先指定的两个身份；测试不得只断言轨迹数量。保留一个 SORT 对照结果，证明用例确实覆盖新增机制。
- `OCSortTracker` 覆盖创建、确认、丢失、OCR 恢复、ORU 恢复、删除边界、非法检测、异常时间戳不改状态、reset 和实例隔离。
- Replay 的默认 SORT 输出逐字节保持兼容；显式选择两种 tracker 均可完成成功案例，非法 tracker 名非零退出。开启可视化时两种 tracker 均可运行。
- Debug 与关闭测试的构建均成功，完整 CTest 通过；现场两组数据从首帧重放至末帧。结果只报告本机固定数据表现，不宣称 Windows、Jetson 或真实身份指标通过。

**范围：** 不实现动态 dt、毫秒生命周期、低置信度第二通道、GIoU/DIoU、ReID、参数搜索、Qt 适配或性能目标认证。

## 第二批完成判定

Task 08–10 分别通过验收后，运行完整 CTest，并分别对合成数据和两组现场数据执行 SORT / OC-SORT 重放。此时可交付“固定步长 OC-SORT + 可视化对照基础设施”；不得宣称 8 fps 时间化优化、现场身份指标、跨平台集成或第一版整体完成。

原计划 Task 11 已由第一批 Task 06 覆盖，不再单列。下一批应从 Task 12 开始，依次处理动态 dt 与时间化生命周期、现场数据定版/身份标注、自动 Regression Test；Qt 和 Jetson 仍应排在算法与数据验收之后。
