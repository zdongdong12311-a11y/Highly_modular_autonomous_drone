# Highly Modular Autonomous Drone

基于 **Livox Mid-360 激光雷达** 的模块化无人机自主避障、建图与导航系统。支持多航点自主飞行、视觉识别抓取投放、多层安全保护机制。

## 硬件配置

| 组件 | 型号 | 备注 |
|------|------|------|
| 机载电脑 | Orange Pi 5 Max | RK3588, 8GB+ RAM |
| 飞控 | CUAV X7+ | PX4 1.13.3 |
| 激光雷达 | Livox Mid-360 | 非重复扫描，360° 视场 |
| 摄像头 (可选) | USB 1080p | MJPG, 用于视觉识别 |
| 舵机爪 (可选) | 铝合金铁爪 + ESP8266 | 串口控制，抓取/投放 |

## 软件环境

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 20.04 (aarch64) |
| ROS | Noetic |
| PX4 | 1.13.3 |
| Python | 3.8+ |
| OpenCV | 4.x (视觉识别模块) |



## 系统架构

```
Livox Mid-360 LiDAR
    │
    ├─→ Livox-SDK2 → livox_ros_driver2 → 原始点云
    │
    ├─→ FAST-LIO2 → /Odometry & /Odom_high_freq
    │       │
    │       ├─→ lidar_to_mavros → /mavros/vision_pose/pose → PX4 EKF2 (视觉位置融合)
    │       │
    │       └─→ /cloud_registered (去畸变三维点云)
    │               │
    │               └─→ EGO-Planner (局部3D避障与轨迹规划)
    │                       │
    │                       └─→ traj_server (轨迹跟踪) → 飞控指令
    │                               │
    │                               └─→ navigation.py → 航点任务 / 控制逻辑
    │
    └─→ MAVROS ↔ PX4 (飞控通信)
```



## 坐标变换 (TF) 树

```
camera_init (FAST-LIO2 实时估计)
    │
    └─→ body (PX4 / MAVROS 发布 / FAST-LIO2 里程计子坐标系)
```

| 变换 | 发布者 | 说明 |
|------|--------|------|
| `camera_init` → `body` | FAST-LIO2 | SLAM 实时估计 |
| `base_link` → `body` | PX4 (MAVROS) | 飞控本体位姿 |

> **重要**: EGO-Planner 需要统一的环境坐标系，确保规划器接收的点云和里程计位于同一坐标系下。

## 目录结构

```
drone/
├── README.md                    ← 本文档
├── navigation.py                ← 自主导航脚本 (纯导航)
├── point.txt                    ← 航点文件 (支持 # 注释)
│
├── 1.mid360-drone/              ← Step 1: LiDAR 驱动 + 状态估计 + PX4 桥接
│   ├── README.md                ← 详细安装指南
│   └── lidar_to_mavros/         ← ROS 包: FAST-LIO2 → PX4 vision_pose 桥接
│       ├── launch/lidar_to_mavros.launch
│       ├── src/lidar_to_mavros.cpp
│       ├── CMakeLists.txt
│       └── package.xml
│
├── 2.ego-planner/               ← Step 2: 3D 轨迹规划与避障 (EGO-Planner)
│   ├── README.md                ← 详细配置指南
│   └── plan_manage/             ← 规划器核心启动与配置文件
│       └── launch/
│           ├── single_run_in_exp.launch
│           └── advanced_param_exp.xml
│
└── Modular_fuctions/            ← 模块化功能集
    ├── opecv_RGB_舵机控制铝合金铁爪/   ← OpenCV 颜色识别 + 舵机爪
    │   ├── opencv_nav_micro.py
    │   ├── R.png / G.png / B.png
    │   └── README.md
    └── rknn-yolov8-master/      ← RK3588 NPU 加速 YOLOv8
        ├── src/                 ← C++ 源码 (三线程流水线)
        ├── launch/
        ├── weights/
        └── README.md
```

## 前置安装

每个步骤的详细安装指南见对应目录的 README:

1. **[1.mid360-drone/README.md](./1.mid360-drone/README.md)** — Mid-360 配网与驱动
   - Livox-SDK2 安装
   - livox_ros_driver2 编译
   - FAST-LIO2 安装 (需修改源码适配 driver2)
   - lidar_to_mavros 编译
   - PX4 EKF2 参数调优

2. **[2.ego-planner/README.md](./2.ego-planner/README.md)** — 3D 轨迹规划与避障
   - EGO-Planner 编译与配置
   - 规划器参数详解

3. **[Modular_fuctions/](./Modular_fuctions/)** — 扩展功能
   - OpenCV 颜色识别 + 舵机爪控制
   - RK3588 NPU 加速 YOLOv8 目标检测

## 快速启动

完成前置安装后:

```bash
# 1. 确保 ROS 环境已 source或写入bashrc
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash --extend
source ~/fast_lio2_ws/devel/setup.bash --extend
source ~/trans_ws/devel/setup.bash --extend
source ~/ego_ws/devel/setup.bash --extend

source ~/.bashrc

# 2. 在终端运行自主导航脚本 
python3 navigation.py              # 纯航点导航
```

### ROS 参数配置

导航脚本支持通过 ROS 参数服务器动态配置 (无需改代码):

```bash
# 通过命令行参数覆盖默认值
python3 navigation.py _takeoff_height:=1.0 _waypoint_timeout:=60.0
```



## 航点文件格式

编辑 `point.txt`，每行一个航点:

```
# 格式: x y z hover_time
# x          - 目标点 X 坐标 (米, 对应全局坐标系)
# y          - 目标点 Y 坐标 (米, 对应全局坐标系)
# z          - 目标高度 (米, 相对起飞点)
# hover_time - 到达后悬停时间 (秒)

0.0  0.0  0.6  2.0
1.0  0.0  0.8  3.0
1.0  1.0  1.0  3.0
0.0  1.0  0.8  3.0
0.0  0.0  0.6  2.0
```

航点文件搜索优先级:
1. ROS 参数 `~waypoint_file`
2. 环境变量 `DRONE_WAYPOINT_FILE`
3. 脚本同目录 `point.txt`
4. `~/point.txt`

## 架构说明与已知限制

### 3D 轨迹规划 (EGO-Planner)

本项目采用 EGO-Planner 替代了传统的 2D 激光切片方案。EGO-Planner 直接处理 FAST-LIO2 的三维点云 (`/cloud_registered`) 进行真 3D 避障规划。

### FAST-LIO2 坐标系说明

官方 hku-mars FAST-LIO2 的坐标系名硬编码在 C++ 源码中:
- 父坐标系: `camera_init`
- 子坐标系: `body`

如需修改，需直接编辑 `src/FAST_LIO/src/laserMapping.cpp` 中约 5 处硬编码字符串，修改后重新编译。

### PX4 EKF2 参数

| 参数 | 建议 | 说明 |
|------|------|------|
| `EKF2_EV_CTRL` | 启用水平/垂直位置和偏航融合 | 开启视觉位置融合 |
| `EKF2_HGT_MODE` | Vision | 高度源使用视觉 |
| `EKF2_GPS_CTRL` | 关闭 GPS 融合 | 室内/无 GPS 场景 |
| `EKF2_EV_DELAY` | 实测 | Mid-360 + FAST-LIO2 延迟需现场测 |
| `EKF2_EV_POS_X/Y/Z` | 实际安装外参 | LiDAR 相对飞控中心的偏移 |
| `EKF2_EVP_NOISE` | 配合调优 | 视觉位置噪声 |
| `EKF2_EVA_NOISE` | 配合调优 | 视觉姿态噪声 |

延迟测量方法:

```bash
rostopic delay /mavros/vision_pose/pose
rostopic delay /mavros/local_position/pose
```

| 问题 | 原因 | 解决 |
|------|------|------|
| EGO-Planner 报无点云或规划失败 | TF 树不完整或里程计无高频输出 | 检查 `/Odom_high_freq` 和 `/cloud_registered` 话题 |
| 起飞后漂移 | OFFBOARD 前设定点未锁位 | 已在代码中修复: 锁死 lock_x/lock_y/lock_yaw |
| 爪子串口连接失败 | 权限不足或设备不存在 | `sudo chmod 666 /dev/ttyUSB0` |
| FAST-LIO2 编译找不到 driver2 | CMAKE_PREFIX_PATH 未设置 | `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/livox_ws/devel` |
