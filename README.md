# 主体mid360-drone

利用 Livox Mid-360 激光雷达实现无人机自主避障、建图和导航。

## 硬件配置

| 组件 | 型号 |
|------|------|
| 机载电脑 | Orange Pi 5 Max |
| 飞控 | CUAV X7+ |
| 飞控固件 | PX4 1.13.3 |
| 激光雷达 | Livox Mid-360 |

## 软件环境

- Ubuntu 20.04
- ROS Noetic
- PX4 1.13.3

## 系统架构

```
Mid-360 LiDAR
    │
    ├─→ Livox-SDK2 → livox_ros_driver2 → 原始点云
    │
    ├─→ FAST-LIO2 → /Odometry
    │       │
    │       ├─→ lidar_to_mavros → /mavros/vision_pose/pose → PX4 EKF2 (视觉位置融合)
    │       │
    │       └─→ /cloud_registered_body (去畸变点云)
    │               │
    │               └─→ pointcloud_to_laserscan → /scan (2D激光切片)
    │                       │
    │                       ├─→ Cartographer 2D SLAM → /map (建图与定位)
    │                       │
    │                       └─→ move_base (global_planner + DWA) → /cmd_vel
    │                               │
    │                               └─→ navigation.py → 航点任务
    │
    └─→ MAVROS ↔ PX4 (飞控通信)
```

## 坐标变换 (TF) 树

```
map ──→ odom (静态变换, 恒等)
  │
  └──→ camera_init (静态变换, 恒等)
          │
          └──→ base_link (静态变换, 恒等)
                  │
                  └──→ body (由 PX4 发布, 估计结果)
```

- `map`: 全局地图坐标系（Cartographer 发布 map→camera_init 的变换）
- `odom`: 里程计坐标系，与 map 对齐（静态恒等变换）
- `camera_init`: Cartographer 的 tracking 发布坐标系
- `base_link`: Cartographer 的 tracking 坐标系（机器人本体）
- `body`: PX4 本体坐标系，用于局部代价地图

## 目录结构

```
drone/
├── README.md                    ← 本文档
├── start.sh                     ← 一键启动脚本
├── navigation.py                ← 自主导航 Python 脚本
├── point.txt                    ← 航点文件 (x y z hover_time)
│
├── 1.mid360-drone/              ← Step 1: LiDAR 驱动 + 状态估计 + PX4 桥接
│   ├── README.md
│   └── lidar_to_mavros/         ← ROS 包: 桥接 FAST-LIO2 里程计到 PX4 vision_pose
│       ├── launch/lidar_to_mavros.launch
│       ├── src/lidar_to_mavros.cpp
│       ├── CMakeLists.txt
│       └── package.xml
│
├── 2.3D_to_2D/                  ← Step 2: 3D 点云转 2D 激光扫描
│   ├── README.md
│   └── point_to_scan.launch
│
├── 3.track_nav/                 ← Step 3: SLAM + 导航
│   ├── README.md
│   ├── cartographer/            ← Cartographer SLAM 配置
│   │   ├── launch/livox.launch
│   │   └── lua/livox.lua
│   └── navigation/              ← ROS Navigation Stack 配置
│       └── move_base/
│           ├── launch/nav_3dto2d.launch
│           ├── config/nav.rviz
│           └── yaml/            ← 规划器参数 (6个文件)
│
└── Modular_fuctions/            ← 模块化功能集
```

## 前置安装

每个步骤的详细安装指南见对应目录的 README：

1. **[1.mid360-drone/README.md](./1.mid360-drone/README.md)**
   - Mid-360 配网与驱动 (Livox-SDK2、livox_ros_driver2)
   - FAST-LIO2 安装与编译（需修改源码以适配 livox_ros_driver2）
   - `lidar_to_mavros` ROS 包编译
   - PX4 EKF2 参数调优

2. **[2.3D_to_2D/README.md](./2.3D_to_2D/README.md)**
   - 安装 `pointcloud_to_laserscan` ROS 包
   - 配置点云→激光扫描参数

3. **[3.track_nav/README.md](./3.track_nav/README.md)**
   - Cartographer 2D SLAM 安装与配置
   - ROS Navigation Stack (move_base) 安装与配置

4. **[Modular_fuctions/](./Modular_fuctions/)**
   - 模块化功能集合，详见各子模块 README

## 快速启动

完成上述步骤的安装后：

```bash
# 一键启动所有节点
./start.sh

# 在新终端中运行自主导航
python3 navigation.py
```

## 航点文件格式

编辑 `point.txt`，每行一个航点：

```
x y z hover_time
```

| 字段 | 说明 |
|------|------|
| `x` | 目标点 X 坐标 (米) |
| `y` | 目标点 Y 坐标 (米) |
| `z` | 目标点 Z 坐标 (米, 高度) |
| `hover_time` | 到达后悬停时间 (秒) |

示例：
```
0.0 0.0 0.8 2.0
2.0 0.0 1.0 3.0
2.0 2.0 1.0 3.0
0.0 2.0 0.8 3.0
0.0 0.0 0.6 2.0
```

## 架构说明与已知限制

### 3D → 2D 转换

`point_to_scan.launch` 从去畸变点云（`/cloud_registered_body`）中提取高度范围 **-0.05m ~ +0.1m** 的水平切片，生成 2D 激光扫描。这意味着：

- 只能检测无人机当前高度附近的障碍物
- 无法感知切片上方或下方的障碍物
- 本质上是 **2D 平面导航避障**，不是真 3D 避障

### TF 坐标系

- `map` 和 `camera_init` 之间由 Cartographer 实时估计
- `camera_init` 到 `base_link` 为静态恒等变换（零位移）
- 局部代价地图使用 `camera_init` 作为全局坐标系
- 全局代价地图使用 `map` 作为全局坐标系

### ⚠️ TF 帧说明

**官方 hku-mars FAST-LIO2** 的坐标系名称是硬编码在 C++ 源码中的：
- 父坐标系：`camera_init`（对应里程计/地图坐标系）
- 子坐标系：`body`（对应无人机本体坐标系）

因此 FAST-LIO2 发布 TF：`camera_init → body`。

**Cartographer** 配置为 `published_frame = "camera_init"`，发布 TF：`map → camera_init`（SLAM 估计结果）。

两条 TF 链互补、不冲突：`map → camera_init → body`。

> ⚠️ 如果你看到网上教程提到 `lidar_odometry_frame_id`、`body_pose_frame_id` 等参数，那是社区个人移植版（如 guzhaoyuan 的 fork）添加的功能，如果使用的话，要把map改成world或者其他，否则会冲突。**官方版本没有这些参数**，不能通过改 yaml 或命令行参数修改坐标系名称。

如需修改 FAST-LIO2 的 TF 坐标系名，需直接编辑 `src/FAST_LIO/src/laserMapping.cpp`，修改以下硬编码字符串：

| 位置 | 原文 | 改为 |
|------|------|------|
| `publish_odometry()` 中 | `"camera_init"` | 你想要的父坐标系名 |
| `publish_odometry()` 中 | `"body"` | 你想要的子坐标系名 |
| 其他相关函数（约 5 处） | `"camera_init"` / `"body"` | 同上 |

修改后重新编译即可生效。

### 安全建议

1. 首次飞行前务必在地面手持测试，确认位姿无漂移
2. 建议先在小范围、无遮挡的场地测试
3. 确保遥控器有失控保护 (Failsafe) 配置
4. 建议添加电池电压监测和低电量返航逻辑
5. 注意 PX4 EKF2_EV_DELAY 参数需实测

## 一键启动脚本 (`start.sh`)

启动所有节点的顺序：
1. `lidar_to_mavros.launch` (MAVROS + Livox驱动 + FAST-LIO2 + 位姿桥接)
2. `point_to_scan.launch` (3D点云→2D激光扫描)
3. `livox.launch` (Cartographer SLAM)
4. `nav_3dto2d.launch` (move_base 规划器 + RViz)

所有节点后台运行，退出时自动清理进程。

## 自主导航脚本 (`navigation.py`)

执行流程：
1. 连接到 MAVROS，等待飞控就绪
2. 切换到 OFFBOARD 模式并解锁
3. 自动起飞到指定高度
4. 读取 `point.txt` 航点文件
5. 依次导航到每个航点（XY 由 move_base 规划，Z 由 PID 控制）
6. 在每个航点悬停指定时长
7. 飞完所有航点后保持悬停

### 航点超时保护

每个航点有 120 秒超时，超时后自动跳转到下一个航点。
