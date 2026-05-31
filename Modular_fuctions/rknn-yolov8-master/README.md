
# RK3588 YOLOv8 目标检测

> **RK3588** 上使用 **NPU** 加速运行 **YOLOv8** 的 C++ 项目  
> 640×640 RKNN 模型推理 + 1920×1080 源图/检测框坐标（letterbox，不拉伸变形）  
> 支持 **ROS1 Noetic** 三线程流水线

---

## ✨ 功能一览

| 功能 | 说明 |
|------|------|
| 🧠 **NPU 加速** | 使用 RKNN API 调用 RK3588 NPU，`rknn_init` / `rknn_run` 封装在 `src/engine/` |
| 📐 **letterbox 处理** | 1920×1080 源图 → 等比填充 → 缩放到 640×640，**不拉伸变形** |
| 🎯 **1080p 检测坐标** | 所有检测框、ROS 消息均为 1920×1080 坐标系，与模型输入分辨率解耦 |
| 🚀 **三线程流水线** | 采集+前处理 → NPU 推理 → 后处理+发布，**重叠执行**提升帧率 |
| 📷 **多种输入** | USB 摄像头 `/dev/videoX`、ROS 图像话题、图片文件、视频文件 |
| 📡 **ROS 输出** | `/detect`（标注图）、`/yolo_msgs`（`vision_msgs/Detection2DArray`） |
| 🖼️ **独立程序** | `yolov8_img`（单图检测）、`yolov8_video`（视频检测） |

---

## 📐 分辨率说明（核心概念）

```
┌─────────────────────────────────────────────────────────────────┐
│  源图（相机/视频）            检测框坐标                      │
│  1920 × 1080  ──────────►  1920 × 1080 (ROS / 画框)        │
│       │                                                       │
│       │ letterbox（等比缩放 + 填充）                            │
│       ▼                                                       │
│  填充图（保持比例）                                             │
│       │                                                       │
│       │ resize                                                 │
│       ▼                                                       │
│  NPU 推理                                                      │
│  640 × 640 (RKNN 模型)   ──►  解码 → 映射回 1080p 坐标       │
└─────────────────────────────────────────────────────────────────┘
```

| 概念 | 默认尺寸 | 含义 |
|------|----------|------|
| **RKNN 模型输入** | **640×640** | NPU 实际推理分辨率（`.rknn` 固定） |
| **源图 / 检测坐标** | **1920×1080** | 相机或 ROS 图像；`/yolo_msgs`、画框均在此坐标系 |
| **显示窗口** | 任意 | `imshow` 仅预览，**不改变**检测坐标 |

> 💡 **关键理解**：模型跑 640×640 就能输出 1080p 的检测坐标，不需要 1080p 的 RKNN 模型。

---

## ⚙️ 系统要求

### 硬件

- **RK3588** 开发板（Orange Pi 5 / ROCK 5B / ROC-RK3588S-PC / 友善 NanoPC-T6 等）
- USB 摄像头 或 MIPI 摄像头

### 软件

| 组件 | 版本建议 |
|------|----------|
| OS | Ubuntu 20.04 / 22.04 **aarch64** |
| OpenCV | 3.2+ 或 4.x（`libopencv-dev`） |
| RKNN Runtime | `librknnrt.so` → 放到 `librknn_api/aarch64/` |
| ROS（可选） | Noetic + `vision_msgs` `cv_bridge` `image_transport` |
| CMake | 3.11+ |

### 运行前检查清单

```bash
# 在 RK3588 板子上执行

# 1. 确认架构
uname -m                    # 应为 aarch64

# 2. 确认 RKNN 运行时库存在
ls -lh librknn_api/aarch64/librknnrt.so

# 3. 确认 640×640 模型存在（如 yolov8s_rk3588.rknn）
ls -lh weights/*.rknn

# 4. 确认 OpenCV 已安装
pkg-config --modversion opencv4

# 5. 若用 ROS
rosversion -d               # noetic
```

---

## 🚀 快速开始

### 1️⃣ 环境准备

```bash
# 在 RK3588 板端项目根目录
chmod +x scripts/*.sh

# 安装系统依赖（OpenCV、ROS 包等）
bash scripts/setup_board.sh

# 获取 RKNN 运行时库（若系统已装 rknpu2）
bash scripts/fetch_rknn_runtime.sh

# 下载测试图片/视频（可选）
bash scripts/download_test_assets.sh
```

### 2️⃣ 准备模型

本仓库**不包含** `.rknn` 模型文件。你需要自己准备 **640×640 输入** 的 RK3588 模型。

**转换流程：**

```bash
# Step 1: 在 PC 上导出 ONNX（使用 Ultralytics）
yolo export model=yolov8s.pt format=onnx imgsz=640

# Step 2: 在 PC 上使用 RKNN-Toolkit2 转 RKNN
#   target_platform='rk3588'
#   input_size_list=[[3, 640, 640]]
#   推荐 INT8 量化

# Step 3: 复制到板子 weights/ 目录
scp yolov8s_rk3588.rknn user@rk3588:~/rknn-yolov8-master/weights/
```

**模型要求：**

| 检查项 | 要求 |
|--------|------|
| 输入张量 | 1 个，NHWC UINT8 RGB |
| 输出张量 | **6 个**（YOLOv8 检测头标准输出） |
| 模型输入尺寸 | **640×640**（与 `infer_width/height` 无关） |
| 目标平台 | RK3588 |

> 详细说明见 [`weights/README.md`](weights/README.md)  
> 官方参考：[rknn_model_zoo YOLOv8](https://github.com/airockchip/rknn_model_zoo/tree/main/examples/yolov8)

### 3️⃣ 编译

#### 独立编译（无 ROS）

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 生成：build/yolov8_img、build/yolov8_video
```

#### ROS 编译

```bash
mkdir -p ~/catkin_ws/src
cp -r /path/to/rknn-yolov8-master ~/catkin_ws/src/rknn_yolov8_ros
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

### 4️⃣ 运行

#### 单图检测（独立程序）

```bash
./build/yolov8_img weights/yolov8s_rk3588.rknn images/bus.jpg
# 输出：result.jpg（保持原图分辨率，框为原图坐标）
```

#### 视频检测（独立程序）

```bash
./build/yolov8_video weights/yolov8s_rk3588.rknn videos/test.mp4 0
# 终端每秒输出 FPS
```

#### ROS Launch（推荐，三线程流水线）

```bash
# 相机模式（默认 640 模型 + 1080p 源图）
roslaunch rknn_yolov8_ros yolov8_camera.launch

# 订阅 ROS 图像话题
roslaunch rknn_yolov8_ros yolov8_ros.launch \
  model_path:=/path/to/model.rknn \
  input_topic:=/camera/image_raw

# 关闭显示窗口（略提速）
roslaunch rknn_yolov8_ros yolov8_camera.launch enable_display:=false
```

#### rosrun 示例

```bash
rosrun rknn_yolov8_ros yolov8_ros_node \
  _model_path:=/path/to/yolov8s_rk3588.rknn \
  _input_mode:=camera \
  _infer_width:=1920 \
  _infer_height:=1080 \
  _enable_display:=false
```

#### 冒烟测试

```bash
bash scripts/smoke_test.sh --model weights/yolov8s_rk3588.rknn
```

---

## 🏗️ 项目结构

```
rknn-yolov8-master/
├── src/                          # 源代码
│   ├── yolov8_ros_node.cpp       # ROS 三线程流水线节点
│   ├── yolov8_img.cpp            # 单图检测程序
│   ├── yolov8_video.cpp          # 视频检测程序
│   ├── engine/                   # RKNN NPU 引擎封装
│   │   ├── engine.h              #   抽象引擎接口 NNEngine
│   │   ├── rknn_engine.h         #   RKEngine 声明
│   │   └── rknn_engine.cpp       #   RKEngine 实现（rknn_init/rknn_run）
│   ├── process/                  # 图像预处理 + 模型后处理
│   │   ├── preprocess.h/.cpp     #   letterbox、cvimg2tensor
│   │   └── postprocess.h/.cpp    #   解码 + NMS
│   ├── task/
│   │   └── yolov8.h/.cpp         # Yolov8Custom 检测流程编排
│   ├── draw/
│   │   └── cv_draw.h/.cpp        # 画检测框
│   ├── types/
│   │   ├── datatype.h            #   张量数据结构
│   │   ├── yolo_datatype.h       #   检测结果结构
│   │   └── error.h               #   错误码
│   └── utils/
│       ├── logging.h             #   日志宏（NN_LOG_xxx）
│       └── engine_helper.h       #   引擎辅助
├── librknn_api/                  # RKNN API
│   ├── include/                  #   rknn_api.h 头文件
│   └── aarch64/
│       ├── README.md
│       └── librknnrt.so          #   RKNN 运行时库（需自备）
├── weights/                      # 放置 .rknn 模型文件
│   └── README.md
├── launch/                       # ROS launch 文件
│   ├── yolov8_camera.launch      #   相机模式
│   └── yolov8_ros.launch         #   Topic 订阅模式
├── scripts/                      # 辅助脚本
│   ├── setup_board.sh            #   板端环境安装
│   ├── fetch_rknn_runtime.sh     #   获取 librknnrt.so
│   ├── download_test_assets.sh   #   下载测试资源
│   └── smoke_test.sh             #   冒烟测试
├── images/                       # 测试图片
├── videos/                       # 测试视频
├── CMakeLists.txt
└── package.xml                   # ROS 包描述
```

---

## 🧩 ROS 接口

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | string | **必填** | `.rknn` 模型文件的绝对或包内路径 |
| `input_mode` | string | `camera` | `camera`（本地摄像头）/ `topic`（订阅话题） |
| `camera_id` | int | `0` | 摄像头设备号，对应 `/dev/video0` |
| `input_topic` | string | `/camera/image_raw` | topic 模式下的订阅话题名 |
| `infer_width` | int | `1920` | **源图/检测框**宽度（不是模型输入尺寸！） |
| `infer_height` | int | `1080` | **源图/检测框**高度（不是模型输入尺寸！） |
| `score_threshold` | float | `0.25` | 置信度阈值 |
| `nms_threshold` | float | `0.25` | NMS IoU 阈值 |
| `enable_display` | bool | `true` | 是否显示 `imshow` 预览窗口 |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/detect` | `sensor_msgs/Image` | 1080p 标注图（BGR） |
| `/yolo_msgs` | `vision_msgs/Detection2DArray` | 检测框（中心+宽高，**1080p 像素坐标**） |

---

## 🔧 架构详解

### 三线程流水线（ROS 节点）

```
┌────────────────────────────────────────────────────────────────────┐
│  Thread1: 采集 / 订阅                                              │
│  ┌─────────────────────────────────────┐                          │
│  │  1. 获取图像帧                      │                          │
│  │  2. resize 到 1920×1080            │                          │
│  │  3. letterbox（等比填充）            │                          │
│  │  4. cvimg2tensor（转 NPU 输入格式）  │                          │
│  └──────────────┬──────────────────────┘                          │
│                 │ infer_queue (队列长度=1)                          │
│                 ▼                                                  │
│  Thread2: NPU 推理（主要瓶颈）                                     │
│  ┌─────────────────────────────────────┐                          │
│  │  rknn_run (640×640)                 │                          │
│  └──────────────┬──────────────────────┘                          │
│                 │ post_queue (队列长度=1)                           │
│                 ▼                                                  │
│  Thread3: 后处理 + 发布                                            │
│  ┌─────────────────────────────────────┐                          │
│  │  1. 解码 NPU 输出（6 路检测头）      │                          │
│  │  2. NMS 非极大值抑制                 │                          │
│  │  3. 映射坐标到 1080p                │                          │
│  │  4. 画框 → /detect                  │                          │
│  │  5. 发布 → /yolo_msgs               │                          │
│  │  6. [可选] imshow 显示              │                          │
│  └─────────────────────────────────────┘                          │
│                                                                   │
│  Ping-Pong Buffer: free_queue_ 管理 2 个 Slot，零拷贝切换           │
└────────────────────────────────────────────────────────────────────┘
```

### 独立程序（串行）

- `yolov8_img`：加载模型 → 读图 → 前处理 → NPU 推理 → 后处理 → 画框 → 保存
- `yolov8_video`：加载模型 → 循环（读帧 → 前处理 → NPU 推理 → 后处理 → 画框 → 显示）

---

## 📊 性能与帧率

### 典型配置：640 模型 + 1080p 源图（推荐）

NPU 只跑 **640×640**，与源图是 1080p 还是 640 无关（仅 CPU 多一步 letterbox）。

| 阶段 | 耗时（约） | 说明 |
|------|------------|------|
| 前处理（1080p letterbox → 640 tensor） | 12–20 ms | CPU，软瓶颈 |
| NPU `rknn_run` | 25–35 ms | **硬瓶颈**，受模型大小和量化影响 |
| 后处理（640 网格解码 + NMS） | 11–23 ms | CPU |
| **ROS 三线程流水线** | **≈ 22–28 FPS** | 三阶段重叠并行 |
| **串行执行**（`yolov8_video`） | **≈ 12–18 FPS** | 无流水线重叠 |

### 不同场景预期帧率

| 使用方式 | 预期 FPS |
|----------|----------|
| ROS + 640 模型 + 1080p 源 + 开窗口 | **20–26** |
| 同上 + `enable_display:=false` | **22–28** |
| `yolov8_video` + 1080p 视频文件 | **12–18** |
| 摄像头直出 640×640 + 640 模型 | **25–30** |
| ❌ **1080p RKNN 模型**（不推荐） | **5–10** |

### 提速建议（按收益排序）

1. 🔄 使用 **YOLOv8n**（更小模型）或 **INT8 量化**
2. 🖥️ 关闭预览窗口：`enable_display:=false`
3. 📉 降低发布频率，或只发 `/yolo_msgs` 不发大图（需改代码）
4. 🎯 若接受较低精度，用 640×640 摄像头源（省去 letterbox）
5. 🔧 确保 NPU 驱动与 `librknnrt.so` 版本匹配

### 实测帧率方法

```bash
# 1. 视频程序（串行，终端会输出 FPS）
./build/yolov8_video weights/yolov8s_rk3588.rknn videos/test.mp4 0

# 2. ROS 话题频率
rostopic hz /detect
rostopic hz /yolo_msgs

# 3. 冒烟脚本
bash scripts/smoke_test.sh --model weights/yolov8s_rk3588.rknn
```

---

## ❓ 常见问题

| 现象 | 原因与解决 |
|------|-----------|
| `RKNN runtime not found` | 缺少 `librknnrt.so` → 执行 `bash scripts/fetch_rknn_runtime.sh` |
| `rknn_init fail` | 模型与 SDK 版本不匹配，或不是 RK3588 模型 |
| 框错位 / 拉伸 | 误将 1080p 直接 resize 成 640×640 送 NPU → 应使用 letterbox 逻辑；检查 `infer_width/height` 是否与相机一致 |
| 帧率很低 | 确认使用的是 **640 模型** 而非 1080p 模型；尝试 `enable_display:=false` |
| 摄像头不是 1080p | 驱动可能忽略设置，代码会自动 resize，但最好在 v4l2 侧设为 1920×1080 |
| 订阅无图 | `input_mode:=topic` 但上游未发布 `input_topic` |
| Windows 编译失败 | 本项目仅支持 **aarch64 Linux**，必须在 RK3588 板端编译 |

---

## ⚠️ 注意事项

1. **识别分辨率 ≠ 窗口分辨率**：`cv::imshow` 仅为预览，检测框与 ROS 消息均以 `infer_width/infer_height` 为准
2. **结束节点**：终端 `Ctrl+C` 即可；`waitKey(1)` 仅刷新窗口，不阻塞退出
3. **模型文件**：仓库**不含** `.rknn`，需自行准备
4. **`infer_width/infer_height`** 是源图/框坐标尺寸，**不要**设成 640

---

## 📜 License

[木兰宽松许可证 第2版](LICENSE)（Mulan PSL v2）

---

## 🙏 致谢

- [Rockchip](https://www.rock-chips.com/) — RKNN SDK
- [Ultralytics](https://github.com/ultralytics/ultralytics) — YOLOv8
- [airockchip](https://github.com/airockchip/ultralytics_yolov8) — 转onnx
- [rknn_model_zoo](https://github.com/airockchip/rknn_model_zoo) — 模型转换参考
