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
    ├─→ FAST-LIO2 → /Odometry
    │       │
    │       ├─→ lidar_to_mavros → /mavros/vision_pose/pose → PX4 EKF2 (视觉位置融合)
    │       │
    │       └─→ /cloud_registered_body (去畸变点云)
    │               │
    │               └─→ pointcloud_to_laserscan → /scan (2D 激光切片)
    │                       │
    │                       ├─→ Cartographer 2D SLAM → /map (建图与定位)
    │                       │
    │                       └─→ move_base (GlobalPlanner + DWA) → /cmd_vel
    │                               │
    │                               └─→ navigation.py → 航点任务 + 高度控制
    │
    └─→ MAVROS ↔ PX4 (飞控通信)
```

## 安全机制

本项目包含多层次安全保护:

| 保护类型 | 触发条件 | 响应动作 |
|----------|----------|----------|
| **低电量降落** | 电池 < 20% (可配置) | 立即降落 |
| **连接断开保护** | MAVROS 断连 | 紧急降落 |
| **位姿超时保护** | 3s 无位姿更新 | 紧急降落 |
| **航点超时跳转** | 单航点 > 120s (可配置) | 跳转下一航点 |
| **起飞超时保护** | 起飞 > 30s 未达目标高度 | 切换 AUTO.LAND |
| **Ctrl+C 安全降落** | 用户中断 | 优雅降落而非直接退出 |
| **速度限幅** | move_base 发出超速指令 | 钳位到安全速度 |

> **首次飞行前务必手持测试**，确认位姿无漂移后再解锁飞行。

## 坐标变换 (TF) 树

```
map ──→ odom (Cartographer 动态变换)
│
└─→ camera_init (Cartographer 实时估计)
    │
    └─→ base_link (静态恒等变换)
        │
        └─→ body (PX4 / MAVROS 发布)
```

| 变换 | 发布者 | 说明 |
|------|--------|------|
| `map` → `odom` → `camera_init` | Cartographer | SLAM 实时估计 |
| `camera_init` → `base_link` | `static_transform_publisher` | 恒等变换 (零位移) |
| `base_link` → `body` | PX4 (MAVROS) | 飞控本体位姿 |

> **重要**: 不要额外静态发布 `map→odom` 或 `map→camera_init`，Cartographer 负责这些动态 TF，重复发布会导致冲突。

## 目录结构

```
drone/
├── README.md                    ← 本文档
├── start.sh                     ← 一键启动脚本 (含健康检查)
├── navigation.py                ← 自主导航脚本 (纯导航)
├── opencv_nav_micro.py          ← 视觉+爪控制+导航 (完整任务)
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
├── 2.3D_to_2D/                  ← Step 2: 3D 点云转 2D 激光扫描
│   ├── README.md
│   └── point_to_scan.launch
│
├── 3.track_nav/                 ← Step 3: SLAM + 导航
│   ├── README.md
│   ├── cartographer/            ← Cartographer 2D SLAM 配置
│   │   ├── launch/livox.launch
│   │   └── lua/livox.lua
│   └── navigation/              ← ROS Navigation Stack 配置
│       └── move_base/
│           ├── launch/nav_3dto2d.launch
│           ├── config/nav.rviz
│           └── yaml/            ← 6 个参数文件
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

2. **[2.3D_to_2D/README.md](./2.3D_to_2D/README.md)** — 点云转激光扫描
   - pointcloud_to_laserscan 安装

3. **[3.track_nav/README.md](./3.track_nav/README.md)** — SLAM + 导航
   - Cartographer 2D SLAM 安装与配置
   - ROS Navigation Stack (move_base) 安装与配置

4. **[Modular_fuctions/](./Modular_fuctions/)** — 扩展功能
   - OpenCV 颜色识别 + 舵机爪控制
   - RK3588 NPU 加速 YOLOv8 目标检测

## 快速启动

完成前置安装后:

```bash
# 1. 确保 ROS 环境已 source
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
source ~/trans_ws/devel/setup.bash

# 2. 启动 roscore (如果尚未运行)
roscore &

# 3. 一键启动所有 ROS 节点 (含健康检查)
chmod +x start.sh
./start.sh

# 4. 在新终端运行自主导航 (二选一)
python3 navigation.py              # 纯航点导航
python3 opencv_nav_micro.py        # 导航 + 视觉识别 + 爪控制
```

`start.sh` 会自动检查:
- roscore 是否运行
- 必要 ROS 包是否已编译并 source
- 关键话题是否出现

退出 `start.sh` (Ctrl+C) 时会自动清理所有后台节点。

### ROS 参数配置

导航脚本支持通过 ROS 参数服务器动态配置 (无需改代码):

```bash
# 通过命令行参数覆盖默认值
python3 navigation.py _takeoff_height:=1.0 _waypoint_timeout:=60.0

# 通过 launch 文件配置
<node pkg="navigation_controller" type="navigation.py" name="nav" output="screen">
    <param name="takeoff_height" value="1.0" />
    <param name="kp_z" value="2.0" />
    <param name="waypoint_xy_tol" value="0.2" />
    <param name="low_battery_threshold" value="15.0" />
</node>
```

#### 核心参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `takeoff_height` | 0.8 | 默认起飞高度 (m) |
| `kp_z` | 1.5 | 高度 P 增益 |
| `kd_z` | 0.0 | 高度 D 增益 (抑制振荡) |
| `max_xy_speed` | 1.5 | 最大水平速度 (m/s) |
| `max_z_speed` | 0.8 | 最大垂直速度 (m/s) |
| `waypoint_xy_tol` | 0.3 | 到达航点 XY 容差 (m) |
| `waypoint_timeout` | 120.0 | 单航点超时 (s) |
| `low_battery_threshold` | 20.0 | 低电量阈值 (%) |
| `waypoint_file` | 自动搜索 | 航点文件路径 |

> 视觉任务脚本 (`opencv_nav_micro.py`) 额外支持: `vision_detect_wp`, `vision_grab_wp`, `vision_search_wps`, `vision_timeout_sec`, `drop_offset_x/y/z`, `camera_id`, `claw_port` 等参数。

## 航点文件格式

编辑 `point.txt`，每行一个航点:

```
# 格式: x y z hover_time
# x          - 目标点 X 坐标 (米, map 坐标系)
# y          - 目标点 Y 坐标 (米, map 坐标系)
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

### 3D → 2D 转换

`pointcloud_to_laserscan` 从去畸变点云 (`/cloud_registered_body`) 中提取高度范围 **-0.05m ~ +0.1m** 的水平切片，生成 2D 激光扫描。

**限制**:
- 只能检测无人机当前高度附近的障碍物
- 无法感知切片上方或下方的障碍物
- 本质上是 **2D 平面导航避障**，不是真 3D 避障

**安全建议**:
- 在开阔场地或已知天花板高度 > 飞行高度 + 1m 的环境中使用
- 避免在低矮障碍物 (桌面、横梁) 附近飞行
- 未来可考虑使用 3D 代价地图 (如 `voxblox` + `mplb`)

### FAST-LIO2 坐标系说明

官方 hku-mars FAST-LIO2 的坐标系名硬编码在 C++ 源码中:
- 父坐标系: `camera_init`
- 子坐标系: `body`

如需修改，需直接编辑 `src/FAST_LIO/src/laserMapping.cpp` 中约 5 处硬编码字符串，修改后重新编译。

> 如果你使用社区 fork (如 guzhaoyuan 的版本)，可能有 `lidar_odometry_frame_id` 等参数。这种情况下需统一 TF 名称，避免与 Cartographer 的 `map` 帧冲突。

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

## 自主导航脚本详解

### `navigation.py` — 纯航点导航

执行流程:
1. 连接 MAVROS，等待飞控就绪
2. 切换 OFFBOARD 模式并解锁
3. 自动起飞到指定高度
4. 读取 `point.txt` 航点
5. 逐航点导航 (XY 由 move_base 规划，Z 由 PD 控制)
6. 到达每个航点后悬停指定时长
7. 全部完成后安全降落

安全特性:
- 低电量监测 → 自动降落
- MAVROS 断连检测 → 紧急降落
- 位姿超时检测 (>3s 无更新) → 紧急降落
- Ctrl+C → 优雅安全降落
- 航点超时 → 自动跳转下一航点

### `opencv_nav_micro.py` — 视觉 + 爪控制 + 导航

在 `navigation.py` 基础上增加:

```
起飞 → 逐航点导航
  → 航点 N (默认4): 第一次视觉识别，锁定目标颜色
  → 航点 M (默认5): 爪子抓取
  → 航点 11/12/13: 限时识别目标颜色
     → 识别成功 → 飞往投放点 → 释放 → 降落
     → 全部失败 → 飞回备降航点 → 释放 → 降落
```

关键航点序号均可通过 ROS 参数配置。

## 一键启动脚本 (`start.sh`)

启动顺序与延迟:

| 序号 | 节点 | 延迟 | 说明 |
|------|------|------|------|
| 1 | lidar_to_mavros.launch | 8s | MAVROS + LiDAR 驱动 + FAST-LIO2 + 位姿桥接 |
| 2 | point_to_scan.launch | 3s | 3D 点云 → 2D 激光扫描 |
| 3 | livox.launch | 3s | Cartographer SLAM |
| 4 | nav_3dto2d.launch | 3s | move_base + RViz |

脚本会:
- 检查 roscore 是否运行
- 验证所有必要 ROS 包是否存在
- 等待关键话题 `/Odometry`、`/mavros/state`、`/scan` 出现
- 检测节点启动后是否立即退出
- Ctrl+C 时有序终止所有子进程

## 安全检查清单

飞行前请逐项确认:

- [ ] 机载电脑与飞控串口连接正常 (`/dev/ttyUSB0` 可访问)
- [ ] Mid-360 网络连接正常 (`ping 192.168.1.1xx` 通)
- [ ] 所有 ROS 包已编译并 source (无 `rospack find` 报错)
- [ ] 手持测试: 位姿无显著漂移 (移动一圈回起点误差 < 10cm)
- [ ] 遥控器失控保护 (Failsafe) 已配置
- [ ] 电池满电 (> 90%)
- [ ] 飞行场地无低矮障碍物 (桌面、横梁等)
- [ ] EKF2_EV_DELAY 已实测并配置
- [ ] 航点文件 `point.txt` 已准备且格式正确

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| `start.sh` 报 roscore 未运行 | roscore 未启动 | 先运行 `roscore` |
| `start.sh` 报 ROS 包未找到 | 未 source 或未编译 | `source ~/xxx/devel/setup.bash` |
| Cartographer 报 `Could not find livox.lua` | launch 文件路径不对 | 使用本项目 `3.track_nav/cartographer/launch/livox.launch` |
| move_base 运行但 `/cmd_vel` 无输出 | TF 树不完整 | `rosrun tf tf_echo map base_link` |
| 建图漂移严重 | 扫描匹配参数不当 | 调整 `livox.lua` 中的 `translation_weight` 和 `min_score` |
| 起飞后漂移 | OFFBOARD 前设定点未锁位 | 已在代码中修复: 锁死 lock_x/lock_y/lock_yaw |
| 爪子串口连接失败 | 权限不足或设备不存在 | `sudo chmod 666 /dev/ttyUSB0` |
| FAST-LIO2 编译找不到 driver2 | CMAKE_PREFIX_PATH 未设置 | `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/livox_ws/devel` |

## License

MIT
