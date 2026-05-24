# Step 2: 3D 点云转 2D 激光扫描

将 FAST-LIO2 输出去畸变后的 3D 点云转换为 2D 激光扫描数据，供 Cartographer 2D SLAM 和 Navigation Stack 使用。

---

## 原理

FAST-LIO2 输出的 `/cloud_registered_body` 是已去畸变的 3D 点云。`pointcloud_to_laserscan` 节点从中提取一个**水平薄层**（默认高度范围 -0.05m ~ +0.1m），生成标准的 `sensor_msgs/LaserScan` 消息。

**限制：** 只能检测无人机当前高度附近的障碍物。该方案本质上是 **2D 平面导航避障**。

---

## 安装

```bash
mkdir -p 3D_to_2D_ws/src
cd 3D_to_2D_ws/src
git clone https://github.com/ros-perception/pointcloud_to_laserscan.git -b lunar-devel
cd ..
catkin_make
source devel/setup.bash
```

---

## 配置

复制本项目中的 launch 文件到工作空间：

```bash
cp /path/to/point_to_scan.launch 3D_to_2D_ws/src/pointcloud_to_laserscan/launch/
```

### 参数说明 (`point_to_scan.launch`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cloud_in` | `/cloud_registered_body` | 输入点云话题 (FAST-LIO2 去畸变输出) |
| `scan` | `/scan` | 输出激光扫描话题 |
| `min_height` | -0.05 | 点云切片最小高度 (m)，低于此的点被过滤 |
| `max_height` | 0.1 | 点云切片最大高度 (m)，高于此的点被过滤 |
| `angle_min` | -3.14159 | 扫描起始角度 (rad)，-π |
| `angle_max` | 3.14159 | 扫描结束角度 (rad)，+π |
| `angle_increment` | 0.001 | 角度分辨率 (rad)，约 0.057° |
| `range_min` | 0.1 | 最小有效距离 (m) |
| `range_max` | 50.0 | 最大有效距离 (m) |
| `scan_time` | 0.1 | 扫描周期 (s) |
| `concurrency_level` | 0 | 并行度 (0=自动检测CPU核心) |

### 调整建议

- **高度范围 (`min_height`/`max_height`)**: 根据无人机实际飞行高度调整。值越小，障碍物检测越精确但也越容易漏检。建议 ±0.1m。
- **角度分辨率 (`angle_increment`)**: 0.001 rad 较密集，如性能不足可增大到 0.003。

---

## 运行

```bash
roslaunch pointcloud_to_laserscan point_to_scan.launch
```

### 验证

```bash
# 确认 /scan 话题有数据输出
rostopic echo /scan | head -20

# 在 RViz 中查看
# Add → By topic → /scan → LaserScan
```
