# 跟踪公共接口契约

本文件定义公共数据与 `ITracker` 接口。Task 06 的 `SortTracker` 已落实本文件所列的首批运行时契约。

## 公共头文件

- `tracking/types.h`：`BBox`、`Detection`、`TrackState`、`TrackResult`。
- `tracking/tracker_config.h`：`TrackerConfig`。
- `tracking/itracker.h`：`ITracker`。

这些头文件只依赖 C++ 标准库，不暴露 Qt、OpenCV、Eigen 或平台类型。

## 坐标与检测

`BBox` 使用像素坐标 `x1`、`y1`、`x2`、`y2`。宽度和高度分别为 `x2 - x1` 与 `y2 - y1`，不加 1。有效框的四个值均为有限数值，且必须满足 `x2 > x1`、`y2 > y1`；允许负坐标，核心不裁剪图像边界。

`Detection::confidence` 必须为有限的 `[0, 1]` 数值。首批仅跟踪单一行人类别，`classId` 仅保留输入数据；调用方负责筛选类别。

`SortTracker::update()` 跳过无效框、非有限置信度、置信度不在 `[0,1]` 内或低于 `detectionThreshold` 的检测，不得以它们创建或更新轨迹。

## 配置

`TrackerConfig` 的默认值为：

| 字段 | 默认值 | 有效范围 |
| --- | ---: | --- |
| `detectionThreshold` | 0.5 | 有限的 `[0, 1]` |
| `iouThreshold` | 0.3 | 有限的 `[0, 1]` |
| `minHits` | 2 | 不小于 1 |
| `maxAgeFrames` | 8 | 不小于 0 |
| `nominalFps` | 8.0 | 有限正数 |

`SortTracker` 在构造时验证这些范围；非法配置抛出 `std::invalid_argument`。`maxAgeFrames` 是基础 SORT 的帧数策略，不表示后续时间化超时。

## ITracker

`update(const std::vector<Detection>& detections, std::int64_t timestampMs)` 每输入帧调用一次；漏检帧传入空列表。时间戳属于同一图片流，首帧必须非负，之后必须严格递增。`SortTracker` 违反时间戳契约时抛出 `std::invalid_argument` 且不改变内部状态。

一个实例仅服务一条图片流，调用方串行调用 `update()` 和 `reset()`。`reset()` 清空轨迹、时间戳和 ID 计数；重置后的下一条轨迹从 ID 1 开始。ID 在单个实例、单次会话内从 1 递增，删除后不复用；跨 `reset()` 的业务身份由调用方区分。

`TrackResult::bbox` 是本帧估计框，`confidence` 是最近一次匹配检测的置信度。`velocityX` 和 `velocityY` 的单位为像素/秒；首批实现用固定步长速度和 `nominalFps` 换算，不表示动态时间估计。`age` 包含创建帧并在每次 `update()` 时增加；`lostFrames` 表示连续未匹配帧数，匹配后归零。

`TrackState` 仅包含 `Tentative`、`Confirmed` 与 `Lost`。首次命中计入 `hitStreak`；连续 `minHits` 次命中后为 `Confirmed`。Tentative 首次漏检即删除；Confirmed 漏检后为 Lost；Lost 的 `lostFrames > maxAgeFrames` 时删除；Lost 再次匹配后立即恢复 Confirmed。输出包含未删除的三种状态，按 `trackId` 升序排列。
