# Step 1: LiDAR 驱动 + 状态估计 + PX4 桥接

| 项目 | 内容 |
|------|------|
| Linux 版本 | Ubuntu 20.04 |
| ROS 版本 | Noetic |
| 机载电脑 | Orange Pi 5 Max |
| 激光雷达 | Livox Mid-360 |
| 飞控 | CUAV X7+ (PX4 1.13.3) |

---

## 一、运行 Mid-360

### 1. 硬件连接

将 Mid-360 的一分三航空线的网线接口插入机载电脑网口，然后给 Mid-360 上电。

### 2. 配网

Mid-360 默认 IP 为 `192.168.1.1xx`（xx 为 SN 码后两位），机载电脑需配置同网段静态 IP。

```bash
# 查看可用网卡
ls /sys/class/net/
# 常见: enp0s3、eth0、enP3p49s0

# 配置静态 IP（替换 enP3p49s0 为你的网卡名）
sudo ip addr add 192.168.1.50/24 dev enP3p49s0
sudo ip link set enP3p49s0 up

# 测试连接（xx 为 SN 码后两位）
ping 192.168.1.1xx
```

也可使用 Ubuntu Network Manager 图形界面配置。

### 3. 安装 Livox-SDK2

```bash
mkdir -p livox_ws/3rd_party
cd livox_ws/3rd_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

**配置 Mid-360 参数：**

编辑 `livox_ws/3rd_party/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json`，将 `host_ip` 改为 `192.168.1.50`。

**运行测试：**

```bash
cd livox_ws/3rd_party/Livox-SDK2/build/samples/livox_lidar_quick_start
./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/mid360_config.json
```

运行成功会有数据流持续输出。如无数据，检查 IP 配置。

### 4. 安装 livox_ros_driver2

```bash
cd livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
./build.sh ROS1
```

**修改雷达参数：**

编辑 `livox_ws/src/livox_ros_driver2/config/MID360_config.json`:

- `host_net_info` 下的所有 IP 改为 `192.168.1.50`
- `lidar_configs` 下的 IP 改为 `192.168.1.1xx`（xx 为 SN 码后两位）

**运行测试：**

```bash
source livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

打开 RViz 应能看到点云。

---

## 二、安装 FAST-LIO2

### 1. 下载与编译

```bash
mkdir -p fast_lio2_ws/src
cd fast_lio2_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
```

**适配 livox_ros_driver2：** FAST-LIO2 默认依赖 livox_ros_driver，需修改以下文件，将所有 `livox_ros_driver` 替换为 `livox_ros_driver2`：

| 文件 | 修改项 |
|------|--------|
| `src/FAST_LIO/CMakeLists.txt` | `find_package` 中的依赖名 |
| `src/FAST_LIO/src/laserMapping.cpp` | `#include` 路径 |
| `src/FAST_LIO/src/preprocess.h` | `#include` 路径 |
| `src/FAST_LIO/src/preprocess.cpp` | `#include` 路径 |

**编译：**

```bash
cd fast_lio2_ws
catkin_make
```

如果提示找不到 `livox_ros_driver2`：

```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/livox_ws/devel
```

> 如果未安装 Eigen3 和 PCL，请先安装：
> ```bash
> sudo apt install libeigen3-dev libpcl-dev
> ```

### 2. 运行测试

```bash
# Terminal 1: 启动 LiDAR 驱动
roslaunch livox_ros_driver2 msg_MID360.launch

# Terminal 2: 启动 FAST-LIO2
roslaunch fast_lio mapping_mid360.launch
```

确认 `/Odometry` 话题有数据输出。

> **注意：** 
> 1. 为后续避障做准备，建议修改 `mapping_mid360.launch`，去掉其中的 RViz 节点。
> 2. **必须**修改 `config/mid360.yaml`，将 `lidar_odometry_frame_id` 从 `"map"` 改为 `"world"`，否则后文 Cartographer 的 `map` 帧会与 FAST-LIO2 的 `map` 帧冲突，导致 TF 系统混乱：
>    ```yaml
>    common:
>      lidar_odometry_frame_id: "world"   # 改为 world，不与 Cartographer 的 map 冲突
>      body_pose_frame_id: "body"
>    ```
>    也可通过命令行覆盖（不改 yaml）：
>    ```bash
>    roslaunch fast_lio mapping_mid360.launch lidar_odometry_frame_id:=world
>    ```

---

## 三、编译 lidar_to_mavros 桥接节点

### 1. 创建并编译工作空间

```bash
mkdir -p trans_ws/src
cp -r ~/lidar_to_mavros trans_ws/src/
cd trans_ws
catkin_make
source devel/setup.bash
```

### 2. 节点说明

`lidar_to_mavros` 节点桥接 FAST-LIO2 的位姿估计到 PX4：

- **订阅：** `/Odometry` (FAST-LIO2 输出) 和 `/mavros/local_position/pose` (PX4 当前位置)
- **发布：** `/mavros/vision_pose/pose` (PX4 视觉位置融合输入)
- **功能：** 将 FAST-LIO2 的位姿以 vision_pose 形式注入 PX4 EKF2，替代 GPS 实现位置估计

### 3. 节点源码 (`lidar_to_mavros.cpp`)

核心逻辑：

```cpp
// FAST-LIO2 里程计回调 → 提取位姿 → 转为 PoseStamped
// → 发布到 /mavros/vision_pose/pose
// → PX4 内部 EKF2 融合该数据
```

控制台日志使用 `ROS_INFO_THROTTLE` 每秒输出 LiDAR 与 PX4 位姿对比。

---

## 四、配置 .bashrc

将以下内容添加到 `~/.bashrc`，确保每次打开终端时环境变量正确：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash --extend
source ~/fast_lio2_ws/devel/setup.bash --extend
source ~/trans_ws/devel/setup.bash --extend
```

```bash
source ~/.bashrc
```

---

## 五、调优 PX4 飞控参数

使用 QGroundControl 连接飞控，修改以下参数：

| 参数 | 值 | 说明 |
|------|------|------|
| `EKF2_EV_CTRL` | 启用水平+垂直位置+偏航 | 开启视觉位置融合 |
| `EKF2_HGT_MODE` | Vision | 高度源使用视觉 |
| `EKF2_GPS_CTRL` | 全部关闭 | 禁用 GPS 融合 |
| `EKF2_EV_DELAY` | **需实测** | Mid-360 扫描周期 100ms + FAST-LIO2 处理 ≈ 80~150ms |
| `EKF2_EV_POS_X/Y/Z` | 实际安装外参 | LiDAR 相对于飞控中心的位置偏移 |
| `EKF2_EVP_NOISE` | 配合调优 | 视觉位置噪声标准差 |
| `EKF2_EVA_NOISE` | 配合调优 | 视觉姿态噪声标准差 |

**延迟测量方法：**

```bash
# 比较 vision_pose 时间戳和 PX4 本地位置时间戳
rostopic delay /mavros/vision_pose/pose
rostopic delay /mavros/local_position/pose
```

---

## 六、初步测试

```bash
# Terminal 1：启动整个 LiDAR 管线
roslaunch lidar_to_mavros lidar_to_mavros.launch
```

`lidar_to_mavros.launch` 会依次启动：
1. MAVROS (px4.launch)
2. Livox ROS Driver 2 (msg_MID360.launch)
3. FAST-LIO2 (mapping_mid360.launch)
4. lidar_to_mavros 桥接节点

**手持测试：**

1. 观察控制台输出，确认 LiDAR 位姿与 PX4 位姿基本一致
2. 拿着飞机绕一圈，回到起点后观察位姿是否漂移
3. 如有明显漂移，检查 EKF2 参数和 FAST-LIO2 配置# Step 1: LiDAR 驱动 + 状态估计 + PX4 桥接

| 项目 | 内容 |
|------|------|
| Linux 版本 | Ubuntu 20.04 |
| ROS 版本 | Noetic |
| 机载电脑 | Orange Pi 5 Max |
| 激光雷达 | Livox Mid-360 |
| 飞控 | CUAV X7+ (PX4 1.13.3) |

---

## 一、运行 Mid-360

### 1. 硬件连接

将 Mid-360 的一分三航空线的网线接口插入机载电脑网口，然后给 Mid-360 上电。

### 2. 配网

Mid-360 默认 IP 为 `192.168.1.1xx`（xx 为 SN 码后两位），机载电脑需配置同网段静态 IP。

```bash
# 查看可用网卡
ls /sys/class/net/
# 常见: enp0s3、eth0、enP3p49s0

# 配置静态 IP（替换 enP3p49s0 为你的网卡名）
sudo ip addr add 192.168.1.50/24 dev enP3p49s0
sudo ip link set enP3p49s0 up

# 测试连接（xx 为 SN 码后两位）
ping 192.168.1.1xx
```

也可使用 Ubuntu Network Manager 图形界面配置。

### 3. 安装 Livox-SDK2

```bash
mkdir -p livox_ws/3rd_party
cd livox_ws/3rd_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

**配置 Mid-360 参数：**

编辑 `livox_ws/3rd_party/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json`，将 `host_ip` 改为 `192.168.1.50`。

**运行测试：**

```bash
cd livox_ws/3rd_party/Livox-SDK2/build/samples/livox_lidar_quick_start
./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/mid360_config.json
```

运行成功会有数据流持续输出。如无数据，检查 IP 配置。

### 4. 安装 livox_ros_driver2

```bash
cd livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
./build.sh ROS1
```

**修改雷达参数：**

编辑 `livox_ws/src/livox_ros_driver2/config/MID360_config.json`:

- `host_net_info` 下的所有 IP 改为 `192.168.1.50`
- `lidar_configs` 下的 IP 改为 `192.168.1.1xx`（xx 为 SN 码后两位）

**运行测试：**

```bash
source livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

打开 RViz 应能看到点云。

---

## 二、安装 FAST-LIO2

### 1. 下载与编译

```bash
mkdir -p fast_lio2_ws/src
cd fast_lio2_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
```

**适配 livox_ros_driver2：** FAST-LIO2 默认依赖 livox_ros_driver，需修改以下文件，将所有 `livox_ros_driver` 替换为 `livox_ros_driver2`：

| 文件 | 修改项 |
|------|--------|
| `src/FAST_LIO/CMakeLists.txt` | `find_package` 中的依赖名 |
| `src/FAST_LIO/src/laserMapping.cpp` | `#include` 路径 |
| `src/FAST_LIO/src/preprocess.h` | `#include` 路径 |
| `src/FAST_LIO/src/preprocess.cpp` | `#include` 路径 |

**编译：**

```bash
cd fast_lio2_ws
catkin_make
```

如果提示找不到 `livox_ros_driver2`：

```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/livox_ws/devel
```

> 如果未安装 Eigen3 和 PCL，请先安装：
> ```bash
> sudo apt install libeigen3-dev libpcl-dev
> ```

### 2. 运行测试

```bash
# Terminal 1: 启动 LiDAR 驱动
roslaunch livox_ros_driver2 msg_MID360.launch

# Terminal 2: 启动 FAST-LIO2
roslaunch fast_lio mapping_mid360.launch
```

确认 `/Odometry` 话题有数据输出。

> **注意：** 为后续避障做准备，建议修改 `mapping_mid360.launch`，去掉其中的 RViz 节点。

---

## 三、编译 lidar_to_mavros 桥接节点

### 1. 创建并编译工作空间

```bash
mkdir -p trans_ws/src
cp -r ~/lidar_to_mavros trans_ws/src/
cd trans_ws
catkin_make
source devel/setup.bash
```

### 2. 节点说明

`lidar_to_mavros` 节点桥接 FAST-LIO2 的位姿估计到 PX4：

- **订阅：** `/Odometry` (FAST-LIO2 输出) 和 `/mavros/local_position/pose` (PX4 当前位置)
- **发布：** `/mavros/vision_pose/pose` (PX4 视觉位置融合输入)
- **功能：** 将 FAST-LIO2 的位姿以 vision_pose 形式注入 PX4 EKF2，替代 GPS 实现位置估计

### 3. 节点源码 (`lidar_to_mavros.cpp`)

核心逻辑：

```cpp
// FAST-LIO2 里程计回调 → 提取位姿 → 转为 PoseStamped
// → 发布到 /mavros/vision_pose/pose
// → PX4 内部 EKF2 融合该数据
```

控制台日志使用 `ROS_INFO_THROTTLE` 每秒输出 LiDAR 与 PX4 位姿对比。

---

## 四、配置 .bashrc

将以下内容添加到 `~/.bashrc`，确保每次打开终端时环境变量正确：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash --extend
source ~/fast_lio2_ws/devel/setup.bash --extend
source ~/trans_ws/devel/setup.bash --extend
```

```bash
source ~/.bashrc
```

---

## 五、调优 PX4 飞控参数

使用 QGroundControl 连接飞控，修改以下参数：

| 参数 | 值 | 说明 |
|------|------|------|
| `EKF2_EV_CTRL` | 启用水平+垂直位置+偏航 | 开启视觉位置融合 |
| `EKF2_HGT_MODE` | Vision | 高度源使用视觉 |
| `EKF2_GPS_CTRL` | 全部关闭 | 禁用 GPS 融合 |
| `EKF2_EV_DELAY` | **需实测** | Mid-360 扫描周期 100ms + FAST-LIO2 处理 ≈ 80~150ms |
| `EKF2_EV_POS_X/Y/Z` | 实际安装外参 | LiDAR 相对于飞控中心的位置偏移 |
| `EKF2_EVP_NOISE` | 配合调优 | 视觉位置噪声标准差 |
| `EKF2_EVA_NOISE` | 配合调优 | 视觉姿态噪声标准差 |

**延迟测量方法：**

```bash
# 比较 vision_pose 时间戳和 PX4 本地位置时间戳
rostopic delay /mavros/vision_pose/pose
rostopic delay /mavros/local_position/pose
```

---

## 六、初步测试

```bash
# Terminal 1：启动整个 LiDAR 管线
roslaunch lidar_to_mavros lidar_to_mavros.launch
```

`lidar_to_mavros.launch` 会依次启动：
1. MAVROS (px4.launch)
2. Livox ROS Driver 2 (msg_MID360.launch)
3. FAST-LIO2 (mapping_mid360.launch)
4. lidar_to_mavros 桥接节点

**手持测试：**

1. 观察控制台输出，确认 LiDAR 位姿与 PX4 位姿基本一致
2. 拿着飞机绕一圈，回到起点后观察位姿是否漂移
3. 如有明显漂移，检查 EKF2 参数和 FAST-LIO2 配置
