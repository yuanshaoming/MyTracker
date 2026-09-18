# Detection CSV Replay 格式

`tracker_replay` 通过固定 CSV 重放 SORT 或固定步长 OC-SORT，不读取图片、检测模型或 GUI 数据。

```sh
tracker_replay --frames <frames.csv> --detections <detections.csv> --output <tracks.csv> [--tracker sort|ocsort]
```

`--tracker` 省略时为 `sort`，保持既有输出行为；`ocsort` 选择 `OCSortTracker`。工具使用默认 `TrackerConfig`。输入必须为带表头的 UTF-8 数值 CSV；格式不支持引号、转义逗号或电子表格公式。

## 输入

`frames.csv` 表头固定为：

```text
frame_id,timestamp_ms
```

每个 `frame_id` 非负且严格递增；`timestamp_ms` 非负且严格递增。帧表列出每一帧，因此检测为空的帧也会重放。

`detections.csv` 表头固定为：

```text
frame_id,x1,y1,x2,y2,confidence,class_id
```

检测 `frame_id` 非负且非递减，相同帧可连续出现多行，且必须引用 `frames.csv` 中的帧。框必须是有限数值且满足 `x2>x1`、`y2>y1`；置信度必须为有限 `[0,1]` 数值。

工具在启动重放前读取并验证所有输入。列数、数值、顺序、时间戳、框和帧引用出错时，会以 `文件路径:行号:原因` 输出错误并非零退出，且不会创建输出文件。

## 输出

`tracks.csv` 表头固定为：

```text
frame_id,timestamp_ms,track_id,x1,y1,x2,y2,confidence,state,velocity_x,velocity_y,age,lost_frames
```

每行对应某个输入帧中所选 Tracker 的 `update()` 返回的一条轨迹。无轨迹帧没有结果行，但仍由 `frames.csv` 保留。状态固定为 `Tentative`、`Confirmed` 或 `Lost`；记录按帧表顺序和 `track_id` 升序输出。浮点数固定为 6 位小数，使用小数点；相同输入、相同 tracker 选择保证同环境同输入可逐字节比较。

`testdata/replay/` 的样本为合成数据，涵盖单人直线、短时漏检恢复、远离双人和不均匀时间间隔；它不代表现场录制或真实身份指标。
