# 第一批 Codex Task

## 使用方式

本文件是待执行的任务说明，不是已完成记录。每次只执行用户指定的一项；以下任务当前全部为“未开始”。以项目根目录为工作目录，遵守根目录 [AGENTS.md](../AGENTS.md)。

| 任务 | 内容 | 前置任务 |
|---|---|---|
| Task 01 | CMake、静态库和测试骨架 | 无 |
| Task 02 | 公共类型和接口契约 | 01 |
| Task 03 | Geometry | 02 |
| Task 04 | Hungarian Assignment | 02 |
| Task 05 | KalmanBoxTracker | 03 |
| Task 06 | 最小 SORT | 04、05 |
| Task 07 | Detection CSV Replay | 06 |

推荐按编号执行。Task 07 必须先于后续 OC-SORT 任务完成。首批不开发 Qt、OpenCV 可视化、OC-SORT、动态 dt 或真实设备检测导出功能。

每项交付均包括：实现和对应测试、必要文档更新、实际构建/测试命令与结果、未验证事项。不得仅因写完代码就标记任务完成。

## Task 01：建立最小工程

**目标：** 无 Qt、OpenCV 和 GPU 依赖的静态库可构建，GoogleTest 可由 CTest 运行。

**执行说明：**

1. 检查本机 C++ 编译器、CMake 和依赖可用性，记录实际版本；不要自动安装系统软件。
2. 创建根目录 CMakeLists.txt、最小源文件及测试目录。目标名固定为 `tracking_core`、`tracker_tests`，核心要求 C++17。
3. 使用本地 `find_package` 查找 Eigen3；仅在 `BUILD_TESTING=ON` 时查找 GoogleTest 并注册测试。不要隐式联网获取依赖。
4. 尚无算法时允许一个明确标记的构建占位源文件和一个测试框架冒烟测试；后续实际模块建立时移除这些占位内容。
5. 增加仅涵盖构建产物的 .gitignore，更新 README 的真实构建步骤和依赖要求。

**验收：** Debug 配置、构建、CTest 成功；单独配置 `BUILD_TESTING=OFF` 的构建目录并成功生成静态库；关闭测试时不查找 GoogleTest。报告本机验证，不宣称三平台均通过。

**范围：** 不定义 Tracker 算法，不创建空实现的未来模块，不初始化 Git。

## Task 02：公共类型与接口契约

**目标：** 形成可独立包含、无平台类型泄漏的公共头文件。

**交付：** `include/tracking/types.h`、`tracker_config.h`、`itracker.h` 及接口测试；必要的契约说明放入 `docs/tracker_contract.md`。

**执行说明：**

- 使用 `tracking` 命名空间，定义 BBox、Detection、TrackState、TrackResult，字段以原计划为基础，所有成员均有确定的初始化值。
- ITracker 提供虚析构、`update(const std::vector<Detection>&, std::int64_t timestampMs)` 和 `reset()`。
- BBox 必须是有限数值且 `x2>x1、y2>y1`；允许负坐标，核心不裁剪图像边界。confidence 为有限的 [0,1] 数值。
- 非法检测由 update 跳过；非法配置抛出 `std::invalid_argument`。时间戳首帧非负、后续严格递增，异常抛出 `std::invalid_argument` 且不改变状态。首次实现 update 时兑现这些规则。
- 首批只接收单一行人类别；classId 保留为数据字段，不实现多类别跟踪。调用方负责筛选行人。
- TrackResult 的 bbox 为本帧估计框，confidence 为最近一次匹配检测的置信度；velocityX/Y 采用像素/秒，首批通过固定步长速度与 `nominalFps` 换算，不能标为动态时间估计。
- 首批配置仅加入后续 SORT 所需字段：detectionThreshold=0.5、iouThreshold=0.3、minHits=2、maxAgeFrames=8、nominalFps=8。阈值范围 [0,1]、minHits≥1、maxAgeFrames≥0、nominalFps 为有限正数。该帧数为基础 SORT 初始配置，不等价于后续时间化超时。
- age 包括创建帧，每次 update 增加；lostFrames 表示连续未匹配帧数，匹配后归零。
- 每实例 ID 从 1 递增，实例内删除后不复用；reset 清空轨迹、时间戳及 ID 计数，重置后从 1 开始。业务层跨 reset 保存身份时须自行区分会话。

**验收：** 公共头可独立编译，无 Qt/OpenCV/Eigen 公共类型；类型初始化、配置默认值及接口签名测试通过。接口尚未实现的运行时行为只记录为契约，不伪造测试结果。

## Task 03：Geometry

**目标：** 提供可复用的内部几何函数，明确数值边界。

**交付：** `src/geometry.h/.cpp`、`tests/test_geometry.cpp`。

**执行说明：** 实现框有效性、面积、中心、IoU、中心欧氏距离、从旧中心到新中心的单位方向，以及 BBox 与 `[cx,cy,s,r]` 观测量的双向转换。无位移返回零方向。转换拒绝非有限值、非正面积或宽高比，使用明确的失败返回值，不产生 NaN 框；无效框的面积和 IoU 返回 0。固定一种内部失败表示，避免多套错误机制。

**验收：** 完全相同框 IoU=1、不相交及仅接触 IoU=0、已知部分相交的准确比值；覆盖包含关系、负坐标、无效框、零位移及转换往返容差。不要只测试函数能运行。

## Task 04：Hungarian Assignment

**目标：** 在有效候选边中先获得最多匹配，再最小化总代价，并返回双方未匹配索引。

**交付：** `src/hungarian.h/.cpp`、`tests/test_hungarian.cpp`。

**执行说明：**

- 支持矩形代价矩阵和显式有效边掩码；维度表示必须能区分 0×N 与 N×0。
- 禁止边不得进入结果；不能简单“无约束求解后删除超阈值边”，因为这可能丢掉其他可行匹配。
- 明确维度错误及有效边含非有限代价的失败规则；等价最优解保持确定性。
- 输出 matches、unmatchedTracks、unmatchedDetections，索引无重复且完整覆盖输入。

**验收：** 覆盖 0×0、0×N、N×0、方阵、两类矩形矩阵、全禁用边、部分禁用边、相同代价和非法输入。使用小矩阵穷举作为独立最优性对照，验证匹配数量与总代价。加入“事后删除会漏掉可行匹配”的回归样例。

## Task 05：KalmanBoxTracker

**目标：** 完成固定步长 7 维框状态预测和观测更新。

**交付：** 内部 Kalman 头/源文件及 `tests/test_kalman.cpp`；文档记录状态、矩阵及初始协方差选择。

**执行说明：** 状态为 `[cx,cy,s,r,vx,vy,vs]`，观测为 `[cx,cy,s,r]`，每帧固定 dt=1。使用 Eigen，提供初始化、predict、update、读取估计框；通过矩阵求解完成更新，不直接求逆。明确面积预测非正时的处理及无法生成有效框时的失败状态。此任务不分配业务 ID，不实现生命周期、历史观测或动态 dt。

**验收：** 覆盖初始化往返、匀速、停止再移动、连续预测后重新观测、大小变化、非法观测与面积边界；断言输出有限、宽高有效、误差处于明确容差内。增加可独立计算的简单预测/更新数值案例，避免只有宽泛“误差不太大”断言。

## Task 06：最小 SORT 与生命周期

**目标：** 提供实现 ITracker 的 SortTracker，形成可测试的跟踪闭环。

**交付：** `include/tracking/sort_tracker.h`、对应内部实现与关联模块、`tests/test_tracker.cpp` 和合成序列测试。

**执行说明：**

1. 校验时间戳后过滤无效及低置信度检测，预测现有轨迹，建立 IoU 代价 `1-IoU`。只有 IoU≥iouThreshold 的边有效，调用 Task 04 求解。
2. 更新匹配轨迹，为未匹配检测创建轨迹，处理未匹配轨迹。核心不读取文件。
3. 首次命中计入 hitStreak；连续 minHits 次命中后 Confirmed。Tentative 任一漏检即删除；已确认轨迹漏检进入 Lost，lostFrames>maxAgeFrames 时删除。Lost 匹配后立即恢复 Confirmed。
4. 返回全部未删除轨迹，包含 Tentative、Confirmed、Lost；按 trackId 排序，Removed 不持续输出。所有计数、confidence 和 reset 行为符合 Task 02。
5. 保持固定步长模型，即使时间间隔不均匀也不改变 Kalman dt；时间戳此阶段只做契约校验。

**验收：** 单人匀速 ID 恒定、远离双人独立 ID、进入/离开、Tentative 漏检、确认边界、删除边界、短时漏检后在门限允许下恢复原 ID；空输入和全无效检测等价。验证 reset 后结果与新实例一致、实例间互不影响、异常时间戳不修改状态、不均匀时间戳可以输入但不会启用动态 dt。

**范围：** 不保证交叉遮挡身份，不实现 OC-SORT，不添加未使用的 OCSortTracker 外壳。此任务先落实基础生命周期，后续时间化阶段替换帧数删除策略。

## Task 07：Detection CSV Replay

**目标：** 不依赖图片、检测模型或 GUI，通过固定 CSV 可重复运行最小 SORT。

**交付：** `tools/tracker_replay.cpp`、必要解析模块、`docs/replay_format.md`、合成数据及 CLI 集成测试。

**执行说明：**

- 使用两个带表头的 UTF-8 CSV：`frames.csv` 为 `frame_id,timestamp_ms`，`detections.csv` 为 `frame_id,x1,y1,x2,y2,confidence,class_id`。帧表包含每一帧，检测表一帧可有零至多行，因此空检测帧不丢失。
- frame_id 非负且帧表严格递增，timestamp_ms 遵循核心契约；检测记录按 frame_id 非递减，相同帧记录允许连续多行，必须引用帧表中的帧。允许帧号有间隔，不自行补帧。
- 对格式错误、无效数值、缺列及不存在的帧引用报告文件和行号，非零退出；输入文件完全验证后再开始重放，避免产生伪完整结果。格式只需支持本任务的数值 CSV，无需通用电子表格解析器。
- 对每个帧表条目恰好调用一次 update。输出 `tracks.csv`：`frame_id,timestamp_ms,track_id,x1,y1,x2,y2,confidence,state,velocity_x,velocity_y,age,lost_frames`。
- 输出只含该帧返回的轨迹；无轨迹帧无结果行，以帧表保留其存在。状态字符串固定为 Tentative、Confirmed、Lost；按帧和 ID 排序，固定浮点格式与小数点区域设置。
- CLI 约定：`tracker_replay --frames <frames.csv> --detections <detections.csv> --output <tracks.csv>`。使用默认 TrackerConfig；首批不增加配置文件系统。
- 提供单人直线、短时漏检、远离双人和不均匀时间间隔的合成样本。说明样本是合成数据，不虚构现场录制或真实身份指标。

**验收：** 同环境同输入两次输出逐字节一致；包含空帧且验证 age/lostFrames 正确推进；从输出检查单人 ID 稳定及短时漏检恢复。覆盖空检测文件（仅表头）、无轨迹帧、错误列数、非数值、无效框、未知帧、乱序帧和重复时间戳；错误输入非零退出并显示定位信息。CTest 包含至少一个完整 CLI 成功与失败案例。

## 首批完成判定

七项任务分别通过验收后，运行完整测试和一次样本重放；README 更新实际可用功能、命令及平台验证范围。此时交付是“最小 SORT + 重放基础设施”，完整 OC-SORT 和正式平台集成仍属后续工作。
