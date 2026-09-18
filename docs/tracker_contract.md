# 跟踪公共接口契约

本文件定义公共数据与 `ITracker` 接口。Task 06 的 `SortTracker` 已落实本文件所列的首批运行时契约。

## 公共头文件

- `tracking/types.h`：`BBox`、`Detection`、`TrackState`、`TrackResult`。
- `tracking/tracker_config.h`：`TrackerConfig`。
- `tracking/itracker.h`：`ITracker`。
- `tracking/sort_tracker.h`、`tracking/ocsort_tracker.h`：两个 `ITracker` 实现。

这些头文件只依赖 C++ 标准库，不暴露 Qt、OpenCV、Eigen 或平台类型。

## 坐标与检测

`BBox` 使用像素坐标 `x1`、`y1`、`x2`、`y2`。宽度和高度分别为 `x2 - x1` 与 `y2 - y1`，不加 1。有效框的四个值均为有限数值，且必须满足 `x2 > x1`、`y2 > y1`；允许负坐标，核心不裁剪图像边界。

`Detection::confidence` 必须为有限的 `[0, 1]` 数值。首批仅跟踪单一行人类别，`classId` 仅保留输入数据；调用方负责筛选类别。

`SortTracker::update()` 与 `OCSortTracker::update()` 跳过无效框、非有限置信度、置信度不在 `[0,1]` 内或低于 `continuationDetectionThreshold` 的检测。置信度低于 `detectionThreshold` 的检测不能创建轨迹；它仅可在轨迹当前为 Confirmed 时续接该轨迹。若该弱续接后的轨迹进入 Lost，则它不能再作为恢复候选，避免以弱观测延长离场轨迹并接管随后进入的人员。

## 配置

`TrackerConfig` 的默认值为：

| 字段 | 默认值 | 有效范围 |
| --- | ---: | --- |
| `detectionThreshold` | 0.5 | 有限的 `[0, 1]` |
| `continuationDetectionThreshold` | 0.4 | 有限的 `[0, detectionThreshold]` |
| `iouThreshold` | 0.3 | 有限的 `[0, 1]` |
| `minHits` | 2 | 不小于 1 |
| `maxAgeFrames` | 2 | 不小于 0 |
| `nominalFps` | 8.0 | 有限正数 |
| `deltaT` | 3 | 不小于 1 |
| `inertia` | 0.2 | 有限的 `[0, 1]` |

`SortTracker` 与 `OCSortTracker` 在构造时验证这些范围；非法配置抛出 `std::invalid_argument`。`continuationDetectionThreshold` 由两种 Tracker 消费；`deltaT` 与 `inertia` 由 `OCSortTracker` 用于 OCM，`SortTracker` 只验证它们，以保持公共配置契约一致。`maxAgeFrames` 是固定步长的帧数策略，不表示后续时间化超时。默认值为 2，允许至多两帧漏检后恢复；连续第三帧漏检时轨迹删除，避免离场人员的 Lost 轨迹接管随后入场的新人员。

## ITracker

`update(const std::vector<Detection>& detections, std::int64_t timestampMs)` 每输入帧调用一次；漏检帧传入空列表。时间戳属于同一图片流，首帧必须非负，之后必须严格递增。两个 Tracker 违反时间戳契约时均抛出 `std::invalid_argument` 且不改变内部状态。

一个实例仅服务一条图片流，调用方串行调用 `update()` 和 `reset()`。`reset()` 清空轨迹、时间戳和 ID 计数；重置后的下一条轨迹从 ID 1 开始。ID 在单个实例、单次会话内从 1 递增，删除后不复用；跨 `reset()` 的业务身份由调用方区分。

`TrackResult::bbox` 是本帧估计框，`confidence` 是最近一次匹配检测的置信度。`velocityX` 和 `velocityY` 的单位为像素/秒；首批实现用固定步长速度和 `nominalFps` 换算，不表示动态时间估计。`age` 包含创建帧并在每次 `update()` 时增加；`lostFrames` 表示连续未匹配帧数，匹配后归零。

`TrackState` 仅包含 `Tentative`、`Confirmed` 与 `Lost`。首次命中计入 `hitStreak`；连续 `minHits` 次命中后为 `Confirmed`。Tentative 首次漏检即删除；Confirmed 漏检后为 Lost；Lost 的 `lostFrames > maxAgeFrames` 时删除；Lost 再次匹配后立即恢复 Confirmed。输出包含未删除的三种状态，按 `trackId` 升序排列。

`SortTracker` 的唯一关联为预测框 IoU。`OCSortTracker` 先使用预测框 IoU 门限内的 OCM，再对第一轮未匹配项以最后真实 observation 的 IoU 执行 OCR；恢复 Lost 轨迹时执行 ORU。两者使用相同的生命周期、ID、固定步长和输出契约；详情见 [OC-SORT 设计说明](ocsort_design.md)。
