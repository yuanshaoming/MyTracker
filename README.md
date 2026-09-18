# MyTracker

轻量级多目标行人跟踪模块：接收现有检测器输出的边界框，在连续图片流中保持行人 ID。

## 当前状态

Task 01 至 Task 06 已提供最小 SORT 核心，Task 07 已提供固定 CSV 重放工具 `tracker_replay`、合成样本和 CLI 集成测试，Task 08 已完成 OpenCV 可视化与现场 SORT 基线复核。Task 09 至 Task 10 已完成固定步长 OC-SORT：OCM、OCR、ORU 和独立 `OCSortTracker` 已可通过 Replay 与可视化工具选择。可构建 C++17 静态库 `tracking_core`，并在开启测试时构建和运行 GoogleTest 目标 `tracker_tests`。

- [开发协作规则](AGENTS.md)
- [第一批 Codex 任务](docs/CODEX_TASKS_BATCH_01.md)
- [第二批 Codex 任务](docs/CODEX_TASKS_BATCH_02.md)
- [第三批 Codex 任务](docs/CODEX_TASKS_BATCH_03.md)
- [公共接口契约](docs/tracker_contract.md)
- [KalmanBoxTracker 说明](docs/kalman_box_tracker.md)
- [CSV Replay 格式](docs/replay_format.md)
- [SORT 图片序列可视化](docs/visualizer.md)
- [OC-SORT 设计说明](docs/ocsort_design.md)
- [原始实施计划](docs/行人跟踪模块开发实施计划.md)

## 目标与边界

输入约 8 fps，正式平台为 Windows 11 和 NVIDIA Jetson Xavier NX，macOS 用于算法开发和重放。正式应用采用 Qt 5.12，核心库独立于 Qt。

实现路线为：基础数学模块 → Kalman → 最小 SORT → 检测重放验证 → OC-SORT → 8 fps 优化 → 平台集成。

```text
现有检测器 → Detection[] → tracking_core → TrackResult[] → Qt / 业务层
检测 CSV ──→ 重放工具 ────→ tracking_core
```

核心采用 C++17、Eigen3，在 CPU 上运行；使用 CMake 构建和 GoogleTest 测试。OpenCV 仅作为后续可视化工具的可选依赖。第一版不包括 ReID、GPU 跟踪、跨摄像机 ID 和长时间完全遮挡后的身份恢复。

## 规划目录

```text
MyTracker/
├── AGENTS.md
├── README.md
├── docs/                    # 设计、任务与数据格式
├── CMakeLists.txt           # Task 01 起创建
├── include/tracking/        # 标准 C++ 公共接口
├── src/                     # 跟踪与数学实现
├── tests/                   # 单元与序列测试
├── tools/                   # CSV 重放、后续可视化
└── testdata/                # 可重复的合成数据与获准使用的现场数据
```

库目标统一为 `tracking_core`，平台产物为 `tracking_core.lib` 或 `libtracking_core.a`。首批工具目标为 `tracker_replay`，测试目标为 `tracker_tests`。

## 接口约定

公共类型将在 Task 02 定义：`BBox`、`Detection`、`TrackState`、`TrackResult`、`TrackerConfig` 和 `ITracker`。

```cpp
std::vector<TrackResult> update(
    const std::vector<Detection>& detections,
    std::int64_t timestampMs);
void reset();
```

- 边界框为像素坐标 `x1,y1,x2,y2`，宽高使用坐标差，不加 1。
- 时间戳为同一图片流的毫秒时间；允许首帧为 0，后续严格递增。
- 每个输入帧调用一次，漏检帧传入空检测列表。
- 一个实例对应一路流，调用方负责串行访问。
- 状态、速度单位、非法输入及重置规则由首批任务中的接口契约落实。

## 构建与测试

需要支持 C++17 的编译器、CMake 3.16 或更高版本以及本地安装的 Eigen3。开启 `BUILD_TESTING` 时还需要本地安装并能被 CMake 找到的 GoogleTest。工程仅通过 `find_package` 查找这些依赖，不会下载或安装依赖；若 CMake 未在默认路径发现它们，可设置 `CMAKE_PREFIX_PATH`。

本机 Task 01 验证环境：Apple clang 21.0.0、CMake 4.3.2、Eigen3 5.0.1（macOS arm64）。实际 GoogleTest 发现与测试结果以本任务交付报告为准。

```sh
cmake -S . -B build -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

`--config Debug` 适用于多配置生成器；单配置生成器通过 `CMAKE_BUILD_TYPE` 选定构建类型。关闭 `BUILD_TESTING` 后应无需 GoogleTest 即可构建核心库。

重放合成样本：

```sh
./build/tracker_replay \
  --frames testdata/replay/frames.csv \
  --detections testdata/replay/detections.csv \
  --output tracks.csv
```

样本为合成数据；格式和输出列见 [CSV Replay 格式](docs/replay_format.md)。

## 首批开发范围

第一批共有七项任务，沿用原计划 Task 01–07 编号：工程骨架、公共接口、Geometry、Hungarian、Kalman、最小 SORT、CSV Replay。每次执行一项，满足前置条件并验收后再继续。

Task 07 完成后才能形成“固定检测数据 → SORT → 可比较结果”的闭环。OC-SORT 开发必须在此闭环通过后开始。原计划第 6 节要求 Replay 先于 OC-SORT，与此顺序一致。

将以下提示交给 Codex 即可开始首项开发：

```text
阅读根目录 AGENTS.md、README.md 和 docs/CODEX_TASKS_BATCH_01.md。
仅执行 Task 01，先说明实现范围与验证方法，完成后报告实际测试结果。
不要开始后续任务。
```

## 第二批开发范围

第二批包括 Task 08–10：SORT 可视化、Observation-Centric Momentum，以及 Observation-Centric Recovery / Re-Update。现有 `SortTracker` 保留为行为和结果基线；只有 Task 09、Task 10 均通过验收后，才可称为本项目的 OC-SORT 实现。

原计划 Task 11 的轨迹生命周期已由第一批 Task 06 实现，因此不重复安排；Task 10 复用同一契约。动态 dt、时间化超时、带身份真值的现场回归、Qt 和 Jetson 验证不属于第二批。

开始第二批首项开发时使用：

```text
阅读根目录 AGENTS.md、README.md 和 docs/CODEX_TASKS_BATCH_02.md。
仅执行 Task 08，先说明实现范围与验证方法，完成后报告实际测试结果。
不要开始后续任务。
```

## 第三批开发范围

第三批规划 Task 12–14：时间戳驱动与毫秒生命周期、现场测试集定版/身份标注，以及自动 Regression Test。默认固定步长模式保持兼容；只有带身份真值的数据才能形成 ID Switch 和 Track Fragment 验收证据。

开始第三批首项开发时使用：

```text
阅读根目录 AGENTS.md、README.md 和 docs/CODEX_TASKS_BATCH_03.md。
仅执行 Task 12，先说明实现范围与验证方法，完成后报告实际测试结果。
不要开始后续任务。
```

## 后续里程碑与验收

第三批完成后再开展 Windows Qt5.12 TrackingAdapter、Jetson ARM64 构建和性能验证。算法时间语义、现场身份回归和参数证据应在平台接入前稳定下来。

第一版需验证单人 ID 稳定、短时漏检恢复、交叉与遮挡场景、完整 reset、跨平台构建、Qt 接入以及持续运行无内存增长。纯运动跟踪对完全遮挡或观测歧义不能保证身份恢复。

单帧处理时间小于 5 ms 是待测目标。性能报告须注明硬件、构建类型、目标数量、数据集及统计口径。ID Switch 等身份指标需要带真实身份标注的数据，不能仅由检测 CSV 推算。
