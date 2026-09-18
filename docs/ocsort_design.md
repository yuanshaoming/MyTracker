# OC-SORT 设计说明

本文描述固定步长 OC-SORT 的 Observation History、Observation-Centric Momentum（OCM）、Observation-Centric Recovery（OCR）和 Observation-Centric Re-Update（ORU）。`OCSortTracker` 是独立的 `ITracker` 实现；它不会改变 `SortTracker` 的结果。

## 参考映射

- OC-SORT 论文的 Observation-Centric Momentum 对应内部的 `ocsort::associate()`：预测框仍以 IoU 建立可行边，真实观测产生的运动方向只用于有效边的排序。
- 官方仓库固定提交 `8462e7e729a93ccd3bd995c0a79a890336cb3a0b` 中的 `k_previous_obs`、`speed_direction` 和方向项，对应 `ObservationHistory::priorTo()`、`geometry::unitDirection()` 和 `angleScore()`。
- OCR 对应官方流程中第一轮关联后，以 last observation 为框的补充 IoU 关联；ORU 对应恢复真实 observation 后的逐帧重放。实现为基于论文语义的 C++ 独立代码，未复制官方 Python 代码；官方仓库使用 MIT License。它不引入其检测器、Python 运行时或 OpenCV 核心依赖。

参考：[论文](https://openaccess.thecvf.com/content/CVPR2023/html/Cao_Observation-Centric_SORT_Rethinking_SORT_for_Robust_Multi-Object_Tracking_CVPR_2023_paper.html)、[官方关联实现（固定提交）](https://github.com/noahcao/OC_SORT/blob/8462e7e729a93ccd3bd995c0a79a890336cb3a0b/trackers/ocsort_tracker/association.py)、[MIT License](https://github.com/noahcao/OC_SORT/blob/8462e7e729a93ccd3bd995c0a79a890336cb3a0b/LICENSE)。

## Observation history

每条候选轨迹独占一个 `ObservationHistory`。记录项为真实匹配 Detection 的 `BBox` 与该次匹配时的轨迹 `age`；Kalman 预测框、空检测帧和 Lost 占位不写入历史。记录相同 age 时覆盖旧值，`reset()` 清空全部记录，因此新轨迹与重置后的轨迹不会共享方向。

`priorTo(currentAge, deltaT)` 的目标 age 是 `currentAge - deltaT`。若该 age 没有真实 observation，按 `currentAge - (deltaT - 1)` 至 `currentAge - 1` 回退，取第一个存在者；没有任何距当前不超过 `deltaT` 帧的 observation 时返回空。默认窗口为 `deltaT = 3`。这既优先使用约三帧的移动趋势，也能跨短漏检回退；超过窗口的旧 observation 不参与方向项。

坐标始终是 `BBox{x1, y1, x2, y2}`，中心是 `((x1+x2)/2, (y1+y2)/2)`，宽高不加 1。

## OCM 分数与匹配

对每条轨迹，运动方向是 `prior observation -> latest observation`；候选方向是 `prior observation -> current Detection`。两者均使用中心的单位向量。历史不足、任一位移为零、框无效或数值不有限时，方向项为中性 `0`，不产生 NaN 或人造方向。

对非中性方向，先把点积裁剪到 `[-1, 1]`，再计算：

```text
angleScore = (pi / 2 - abs(acos(dot))) / pi
ocmSimilarity = IoU + inertia * detection.confidence * angleScore
```

同向、垂直、反向的 `angleScore` 分别为 `0.5`、`0`、`-0.5`。当 `inertia` 在 `[0, 1]`、confidence 在 `[0, 1]` 时，方向增量在 `[-0.5, 0.5]`；因此完整相似度范围是 `[-0.5, 1.5]`，默认 `inertia = 0.2` 时为 `[-0.1, 1.1]`。关联通过最小化其相反数来最大化该相似度。

IoU 仍是硬门限：`IoU < iouThreshold` 的边标记为无效，方向项绝不使它变成可匹配边。有效边交给现有确定性 Hungarian 求解器，以先最大匹配数、再最大 OCM 相似度的顺序分配；无历史或 `inertia = 0` 时，排序退化为纯 IoU 关联。

## OCR 与 ORU

每帧先以预测框执行 OCM；只对第一轮双方未匹配项执行 OCR。OCR 的轨迹框是该轨迹最后一个真实 observation，而非 Lost 期间的预测框；仍以 `iouThreshold` 作为硬门限，并使用同一确定性 Hungarian 求解。任一轨迹或 Detection 在两轮合计最多匹配一次。

每次真实 Detection 更新成功后，轨迹保存该时刻的 Kalman posterior、真实 observation 框和对应 age。预测、空检测帧和 Lost 状态都不覆盖这个锚点。若 Lost 轨迹在任一关联阶段恢复，ORU 从该 posterior 副本重新开始：设最后真实 observation 的 age 为 `a`，当前 age 为 `b`，则对 `a + 1` 到 `b` 逐帧 `predict()` 与 `update()`；中间 `b-a-1` 个虚拟 observation 的中心、宽和高在首尾真实框之间线性插值，`b` 只使用一次当前 Detection。重放任一步产生无效框或非有限状态时，该轨迹视为不可用，不以旧预测静默替代。

本阶段固定 `dt=1`。`timestampMs` 只做单调校验，不参与预测、插值或删除。`TrackerConfig` 的 `deltaT=3`、`inertia=0.2` 由 `OCSortTracker` 消费；`SortTracker` 仍验证它们的范围，但不改变其 IoU-only 关联。

## 低置信度续接

`detectionThreshold=0.5` 仍是创建新轨迹的门限。`continuationDetectionThreshold=0.4` 允许置信度在 `[0.4, 0.5)` 的有效 Detection 只与当前 Confirmed 轨迹参与关联，不得创建新轨迹，也不得关联 Tentative 或 Lost 轨迹。弱 Detection 更新后，若该轨迹随后进入 Lost，它不再参加第一轮 OCM 或 OCR 的恢复关联；高置信度 observation 才可作为正常恢复锚点。这是针对离场人员残余框与随后入场人员重叠的保护规则，不属于 ReID 或 ByteTrack 检测层。

## 本机现场重放记录

记录日期：2026-09-18。以下命令只生成 `/private/tmp` 下的临时 CSV，不改写现场图片或输入 CSV：

```sh
tracker_replay --frames testdata/capture_20260917_单人/frames.csv --detections testdata/capture_20260917_单人/detections.csv --output /private/tmp/single_sort.csv --tracker sort
tracker_replay --frames testdata/capture_20260917_单人/frames.csv --detections testdata/capture_20260917_单人/detections.csv --output /private/tmp/single_ocsort.csv --tracker ocsort
tracker_replay --frames testdata/capture_20260917_双人/frames.csv --detections testdata/capture_20260917_双人/detections.csv --output /private/tmp/double_sort.csv --tracker sort
tracker_replay --frames testdata/capture_20260917_双人/frames.csv --detections testdata/capture_20260917_双人/detections.csv --output /private/tmp/double_ocsort.csv --tracker ocsort
```

均以默认 `TrackerConfig` 完成并退出 0。统计口径为输出行数、出现过的轨迹 ID 数、进入 Lost 次数和从 Lost 恢复次数：

| 数据集 | Tracker | 输出行数 | 轨迹 ID 数 | 进入 Lost | Lost 恢复 | 观察结果 |
|---|---|---:|---:|---:|---:|---|
| `capture_20260917_单人` | SORT | 199 | 7 | 7 | 0 | 与 OC-SORT 输出逐行相同。 |
| `capture_20260917_单人` | OC-SORT | 199 | 7 | 7 | 0 | 完整重放成功。 |
| `capture_20260917_双人` | SORT | 228 | 7 | 6 | 0 | 与 OC-SORT 存在数值和生命周期差异。 |
| `capture_20260917_双人` | OC-SORT | 231 | 7 | 7 | 0 | 完整重放成功。 |

两组现场数据均没有身份真值；这些结果不表示 ID Switch、准确率、跨平台表现或实际身份质量指标。数值差异只说明该固定输入上 ORU 重放改变了部分状态估计。
