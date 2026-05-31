# Step 3: SLAM + 导航

Cartographer 2D SLAM（建图定位）+ ROS Navigation Stack（路径规划避障）。

---

## 一、Cartographer 2D SLAM

### 1. 安装依赖

```bash
sudo apt-get install -y build-essential protobuf-compiler clang cmake g++ git \
  google-mock libboost-all-dev libcairo2-dev libcurl4-openssl-dev libeigen3-dev \
  libgflags-dev libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev lsb-release \
  ninja-build stow python3-sphinx libgmock-dev libmetis-dev libceres-dev

sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
```

### 2. 下载并安装 Cartographer

```bash
mkdir -p my_carto/src
cd my_carto
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
```
## 3. rosdep

```bash
sudo rosdep init
rosdep update
网不行更新失败的话可以使用小鱼ros
```

### 4. 解决 libabsl-dev 依赖问题

Ubuntu 20.04 的 rosdep 中 `libabsl-dev` 不可用：

```bash
# 执行 rosdep install 会报错:
# ERROR: ... [libabsl-dev] defined as "not available" for OS version [focal]

# 解决方案: 删除 cartographer/package.xml 中的 libabsl-dev 依赖
# 编辑 my_carto/src/cartographer/package.xml，删除第46行 <depend>libabsl-dev</depend>

# 然后重新执行:
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# 手动安装 abseil
src/cartographer/scripts/install_abseil.sh

# 以下命令可能会报找不到包，属正常现象
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
```

### 5. 编译

```bash
catkin_make_isolated --install --use-ninja -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 5. 使用本项目配置
```
cp /path/to/livox.lua my_carto/src/cartographer_ros/cartographer_ros/configuration_files/
cp /path/to/livox.launch my_carto/src/cartographer_ros/cartographer_ros/launch/
# 重新编译
catkin_make_isolated --install --use-ninja -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 6. 配置文件说明

#### `livox.lua` — Cartographer 2D 核心参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `map_frame` | `map` | 全局地图坐标系 |
| `tracking_frame` | `base_link` | Cartographer 跟踪的机器人坐标系 |
| `published_frame` | `camera_init` | 发布的位姿所在坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `provide_odom_frame` | `true` | 由 Cartographer 发布 SLAM 相关 odom/map 变换 |
| `use_imu_data` | `false` | 禁用 IMU（仅用激光扫描匹配） |
| `TRAJECTORY_BUILDER_2D.min_range` | 0.05m | 最小有效扫描距离 |
| `TRAJECTORY_BUILDER_2D.max_range` | 30m | 最大有效扫描距离 |
| `use_online_correlative_scan_matching` | `true` | 使用实时相关扫描匹配提高鲁棒性 |
| `POSE_GRAPH.optimize_every_n_nodes` | 30 | 每 30 个节点优化一次位姿图 |

#### `livox.launch` — 启动配置

| 节点 | 说明 |
|------|------|
| `cartographer_node` | 核心 SLAM 节点，订阅 `/scan`、`/odom`、`/imu` |
| `cartographer_occupancy_grid_node` | 将子图发布为占用网格地图 (分辨率 0.05m) |
| `static_transform_publisher` | 只发布固定变换 `camera_init→base_link`；`map/odom/camera_init` 动态 TF 由 Cartographer 负责 |

### 7. 运行

```bash
source ~/my_carto/install_isolated/setup.bash
cd ~/my_carto
roslaunch cartographer_ros livox.launch
```

---

## 二、ROS Navigation Stack

### 1. 安装依赖

```bash
sudo apt-get install libsdl-image1.2-dev libsdl-dev
sudo apt-get install ros-noetic-tf2-sensor-msgs

# 更新 ROS 密钥（如密钥过期）
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
  --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-noetic-move-base-msgs
```

### 2. 下载并编译 navigation

```bash
mkdir -p ros_nav_ws/src
cd ros_nav_ws/src
git clone https://github.com/ros-planning/navigation.git
cd ..
catkin_make
source devel/setup.bash
```
### 3. 使用本项目配置

```bash
cp ~/path/yaml ~/path/navigation/move_base
cp ~/path/config ~/path/navigation/move_base
cp ~/path/launch ~/path/navigation/move_base
catkin_make
source devel/setup.bash
```

### 4. 配置文件说明

#### 规划器选择 (`nav_3dto2d.launch`)

| 规划器 | 类型 | 说明 |
|--------|------|------|
| 全局规划器 | `global_planner/GlobalPlanner` | Dijkstra 算法，2D 全局路径 |
| 局部规划器 | `dwa_local_planner/DWAPlannerROS` | DWA 动态窗口法，避障 |

#### 参数文件

| 文件 | 说明 |
|------|------|
| `costmap_common_params.yaml` | 通用代价地图参数：机器人半径 0.3m，障碍物层(激光输入)，膨胀层(半径0.3m) |
| `global_costmap_params.yaml` | 全局代价地图：`map` 坐标系，静态地图+障碍物+膨胀 |
| `local_costmap_params.yaml` | 局部代价地图：`camera_init` 坐标系，5×5m 滚动窗口，分辨率 0.03m |
| `global_planner_params.yaml` | Dijkstra 全局规划器参数 |
| `dwa_local_planner_params.yaml` | DWA 参数：最大速度 2.0m/s，仿真时间 3.0s，目标容差 0.2m |
| `move_base_params.yaml` | move_base 通用参数：控制频率 10Hz，规划频率 5Hz，震荡超时 3s |

#### 关键参数说明 (DWA)

| 参数 | 值 | 说明 |
|------|-----|------|
| `max_vel_x` | 1.2 m/s | 最大前向速度 |
| `max_vel_y` | 2.0 m/s | 最大横向速度（无人机可全向移动） |
| `max_vel_trans` | 2.0 m/s | 最大平移速度 |
| `max_vel_theta` | 0.0 | 偏航角速度限制（0 = 不限） |
| `acc_lim_x` | 10.0 m/s² | 前向加速度 |
| `xy_goal_tolerance` | 0.2 m | 到达目标的 XY 容差 |
| `sim_time` | 3.0 s | DWA 轨迹仿真时长 |
| `path_distance_bias` | 32.0 | 路径跟踪权重（越高越贴路径） |
| `goal_distance_bias` | 20.0 | 趋向目标权重 |
| `occdist_scale` | 0.05 | 障碍物代价权重（越高越远离障碍） |

### 5. 运行

```bash
source ~/ros_nav_ws/devel/setup.bash
cd ~/ros_nav_ws
roslaunch move_base 
```

这将启动 move_base 节点和 RViz 可视化界面（含地图、代价地图、全局/局部路径、激光数据）。

---

## 三、TF 坐标变换详情

```
map ──→ odom (Cartographer)
  │
  └─→ camera_init (Cartographer 实时估计/发布)
          │
          └─→ base_link (静态)
                  │
                  └─→ body (PX4 /mavros/local_position/pose)
```

| 变换 | 发布者 | 说明 |
|------|--------|------|
| `map` / `odom` / `camera_init` | Cartographer | SLAM 实时估计的机器人位姿链 |
| `camera_init` → `base_link` | `static_transform_publisher` | 恒等变换，Carter 发布位姿的参考系 |
| `base_link` → `body` | PX4 (MAVROS) | 飞控自身估计的本体位姿 |

> **注意：** 不要额外静态发布 `map→odom` 或 `map→camera_init`。这些变换由 Cartographer 管理，重复发布会造成 TF 冲突。

---

## 四、常见问题

### Cartographer 启动报错

**Q:** `[FATAL] [xxx]: Could not find livox.lua`

**A:** 直接运行本仓库的 `3.track_nav/cartographer/launch/livox.launch`；该文件会从相邻 `lua/` 目录加载 `livox.lua`。

### Navigation 启动后发现 /cmd_vel 无输出

**Q:** move_base 正常运行但无速度指令输出

**A:** 检查 TF 树是否完整。运行 `rosrun tf tf_echo map base_link`，确认所有变换可到达。另外检查激光扫描 `/scan` 话题是否有数据。

### 建图不准/漂移

**Q:** Cartographer 构建的地图与真实场景不符

**A:**
- 检查 `angle_increment` 是否过大（建议 ≤ 0.003 rad）
- 尝试启用 IMU 数据（设置 `use_imu_data = true`）
- 降低 `POSE_GRAPH.optimize_every_n_nodes`（如改为 20）
- 提高 `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight`
