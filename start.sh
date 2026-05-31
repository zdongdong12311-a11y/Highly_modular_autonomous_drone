#!/bin/bash
# =============================================================
# start.sh - 无人机自主导航系统一键启动脚本
#
# 按顺序后台启动以下 ROS 节点:
#   1. lidar_to_mavros      (MAVROS + LiDAR 驱动 + FAST-LIO2 + 位姿桥接)
#   2. pointcloud_to_laserscan (3D 点云转 2D 激光扫描)
#   3. Cartographer SLAM    (2D SLAM 建图与定位)
#   4. move_base            (全局+局部路径规划与避障)
#
# 所有节点后台运行，退出时 (Ctrl+C / 脚本结束) 自动清理。
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDS=()
NAMES=()

# ---- 颜色输出 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No Color

log_info()  { echo -e "${GREEN}[start.sh]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[start.sh]${NC} $*"; }
log_error() { echo -e "${RED}[start.sh]${NC} $*" >&2; }

# ---- 清理函数 ----
cleanup() {
    log_info "正在停止所有 ROS 节点..."
    for i in "${!PIDS[@]}"; do
        pid="${PIDS[$i]}"
        name="${NAMES[$i]:-unknown}"
        if kill -0 "$pid" 2>/dev/null; then
            log_info "停止 ${name} (PID ${pid})..."
            kill "$pid" 2>/dev/null || true
        fi
    done

    # 等待进程退出 (最多 10 秒)
    log_info "等待进程退出..."
    for i in {1..10}; do
        all_stopped=true
        for pid in "${PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                all_stopped=false
                break
            fi
        done
        if $all_stopped; then
            break
        fi
        sleep 1
    done

    # 强制杀死残留进程
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            log_warn "强制杀死 PID ${pid}..."
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    log_info "所有节点已停止。"
}

trap cleanup EXIT INT TERM

# ---- 检查 roscore ----
check_roscore() {
    if ! rostopic list &>/dev/null; then
        log_error "roscore 未运行! 请先启动 roscore。"
        log_error "  运行: roscore"
        exit 1
    fi
    log_info "roscore 已连接。"
}

# ---- 检查必要 ROS 包 ----
check_ros_package() {
    local pkg="$1"
    if ! rospack find "$pkg" &>/dev/null; then
        log_error "ROS 包 '${pkg}' 未找到! 请确认已编译并 source。"
        return 1
    fi
    return 0
}

# ---- 检查必要话题是否存在 (可选，带超时) ----
wait_for_topic() {
    local topic="$1"
    local timeout="${2:-10}"
    log_info "等待话题 ${topic} (超时 ${timeout}s)..."
    for i in $(seq 1 "$timeout"); do
        if rostopic info "$topic" &>/dev/null; then
            return 0
        fi
        sleep 1
    done
    log_warn "话题 ${topic} 在 ${timeout}s 内未出现 (可能正常，继续启动)。"
    return 0
}

# ---- 后台启动节点 ----
launch_bg() {
    local name="$1"
    local delay="$2"
    shift 2

    log_info "启动 ${name}..."
    "$@" &
    local pid=$!
    PIDS+=("$pid")
    NAMES+=("$name")

    # 等待 delay 秒
    sleep "$delay"

    # 检查进程是否仍在运行
    if kill -0 "$pid" 2>/dev/null; then
        log_info "${name} 已启动 (PID ${pid})"
    else
        log_error "${name} 启动后立即退出! (PID ${pid})"
        # 不终止整个脚本，让用户决定
    fi
}

# ===================== 主流程 =====================

log_info "========================================"
log_info "  无人机自主导航系统 - 一键启动"
log_info "========================================"

# 1. 前置检查
check_roscore

log_info "检查必要 ROS 包..."
ALL_OK=true
for pkg in mavros livox_ros_driver2 fast_lio lidar_to_mavros pointcloud_to_laserscan cartographer_ros move_base; do
    if ! check_ros_package "$pkg"; then
        ALL_OK=false
    fi
done

if ! $ALL_OK; then
    log_error "部分 ROS 包缺失，请检查编译和 source 设置。"
    log_error "  source ~/livox_ws/devel/setup.bash"
    log_error "  source ~/fast_lio2_ws/devel/setup.bash"
    log_error "  source ~/trans_ws/devel/setup.bash"
    log_error "  source /opt/ros/noetic/setup.bash"
    exit 1
fi

log_info "所有必要 ROS 包已就绪。"

# 2. 按顺序启动节点

launch_bg "LiDAR-to-MAVROS" 8 \
    roslaunch "$SCRIPT_DIR/1.mid360-drone/lidar_to_mavros/launch/lidar_to_mavros.launch"

# 等待关键话题出现后再继续
wait_for_topic "/Odometry" 15
wait_for_topic "/mavros/state" 10

launch_bg "3D-to-2D-激光转换" 3 \
    roslaunch "$SCRIPT_DIR/2.3D_to_2D/point_to_scan.launch"

wait_for_topic "/scan" 10

launch_bg "Cartographer-SLAM" 3 \
    roslaunch "$SCRIPT_DIR/3.track_nav/cartographer/launch/livox.launch"

launch_bg "move_base-导航" 3 \
    roslaunch "$SCRIPT_DIR/3.track_nav/navigation/move_base/launch/nav_3dto2d.launch"

# 3. 启动完成
log_info "========================================"
log_info "  所有节点已启动!"
log_info "  PID 列表:"
for i in "${!PIDS[@]}"; do
    log_info "    ${NAMES[$i]}: ${PIDS[$i]}"
done
log_info "========================================"
log_info "按 Ctrl+C 停止所有节点。"
log_info "在新终端运行导航脚本:"
log_info "  python3 $SCRIPT_DIR/navigation.py"
log_info "  python3 $SCRIPT_DIR/opencv_nav_micro.py  # (含视觉+爪控制)"

# 4. 等待任意子进程退出
wait -n 2>/dev/null || wait

# 如果有进程先退出，提示
log_warn "有节点已退出，清理剩余进程..."
