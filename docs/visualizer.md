# SORT 图片序列可视化

`tracker_visualizer` 是仅用于检查固定图片序列和现有 `SortTracker` 输出的可选工具。它不属于 `tracking_core`，不修改跟踪算法，也不读取检测模型。

## 构建

默认 `BUILD_TRACKER_VISUALIZER=OFF`；此时 CMake 不查找 OpenCV，核心库、`tracker_replay` 和测试的依赖不变。

```sh
cmake -S . -B build -DBUILD_TESTING=ON -DBUILD_TRACKER_VISUALIZER=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

本机安装了可被 CMake 发现的 OpenCV 后，才可开启工具：

```sh
cmake -S . -B build-visualizer -DBUILD_TRACKER_VISUALIZER=ON
cmake --build build-visualizer --target tracker_visualizer
```

若 OpenCV 安装目录不在默认搜索路径，配置时传入其包含 `OpenCVConfig.cmake` 的目录，例如 `-DOpenCV_DIR=/path/to/lib/cmake/opencv4`。

开启时只为 `tracker_visualizer` 查找并链接 OpenCV 的 `core`、`highgui`、`imgcodecs` 和 `imgproc` 组件。

## 运行

```sh
tracker_visualizer \
  --frames testdata/capture_20260917_单人/frames.csv \
  --detections testdata/capture_20260917_单人/detections.csv \
  --images testdata/capture_20260917_单人/images \
  --tracker ocsort
```

追加 `--auto-play` 时，工具不创建窗口，按帧表顺序处理完整序列后退出；该模式用于自动验证连续帧处理，不替代人工画面检查。

`--tracker sort|ocsort` 可选择算法，省略时为 `sort`，保持原有可视化行为。

工具复用 `tracker_replay` 的 CSV 解析和全部输入校验，启动重放前完成 CSV 验证。图片按 `images/<六位十进制 frame_id>.jpg` 读取；缺图、解码失败或与首帧尺寸不一致时，工具会显示具体 `frame_id` 并非零退出。每次前进到新帧时，恰好调用一次所选 Tracker 的 `update()`。

画面标记如下：Detection 为绿色；Confirmed 轨迹为蓝色；Lost 轨迹为黄色；Tentative 轨迹为橙色。绿色框展示 CSV 中的原始检测，未表示它一定被核心使用：置信度低于 `0.4` 的框会被跳过，`[0.4, 0.5)` 的框仅可续接当前 Confirmed 轨迹，不能创建新轨迹。轨迹框会显示 ID 和状态。每个 ID 在可视化层保存最近 24 个中心点，以白线绘制历史；该历史不传入核心算法。

| 按键 | 操作 |
|---|---|
| Space | 暂停或继续播放 |
| Right | 暂停并前进一帧 |
| R | 清空可视化历史、重置 Tracker，并从首帧重新开始 |
| D | 显示或隐藏 Detection |
| T | 显示或隐藏轨迹框和文字 |
| H | 显示或隐藏轨迹历史 |
| Esc | 退出 |

## 现场 SORT 基线记录

记录日期：2026-09-18。配置为默认 `TrackerConfig`（`detectionThreshold=0.5`、`continuationDetectionThreshold=0.4`、`iouThreshold=0.3`、`minHits=2`、`maxAgeFrames=2`、`nominalFps=8`），使用 `tracker_replay` 生成临时输出统计；原始图片、CSV 和生成输出均未改写或提交。

| 数据集 | 帧数 | Replay 退出状态 | 创建的轨迹 ID 数 | 进入 Lost 次数 | 从 Lost 恢复次数 | 人工画面观察 |
|---|---:|---:|---:|---:|---:|---|
| `capture_20260917_单人` | 275 | 0 | 7 | 7 | 0 | `--auto-play` 已从首帧至末帧成功；窗口复核未发现明显跟踪异常。 |
| `capture_20260917_双人` | 279 | 0 | 7 | 6 | 0 | `--auto-play` 已从首帧至末帧成功；第 34 帧创建 ID 2；第 85 帧低置信度离场框续接原 ID，入场人员创建新 ID。 |

“进入 Lost”和“从 Lost 恢复”按同一 `track_id` 的相邻输出状态转换计数；它们只描述该固定输入下的 SORT 生命周期，并非身份指标。两组数据没有身份真值，因此本记录不报告 ID Switch、准确率或任何身份质量结论。OpenCV 4.14.0 构建通过，两组图片均已通过工具在窗口创建前执行的缺图、解码和尺寸一致性预检，并以 `--auto-play` 成功处理完整序列。窗口模式已人工复核显示、快捷键、重置和结束行为。双人序列中，第 31 帧的弱检测只续接当时的 Confirmed ID 1；该轨迹在第 32、33 帧 Lost 后不能据第 34 帧的新高置信度检测恢复，因此第 34 帧创建 ID 2。
