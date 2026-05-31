#!/usr/bin/env python3
"""
opencv_nav_micro.py - 视觉识别 + 舵机爪控制 + 自主导航

在 navigation.py 的基础上，集成 OpenCV 颜色识别和 ESP8266 串口爪控制，
实现 "识别 -> 抓取 -> 运输 -> 投放" 的完整任务闭环。

任务流程:
  起飞 -> 逐航点导航
    -> 航点 N (默认4): 第一次视觉识别，锁定目标颜色
    -> 航点 M (默认5): 爪子抓取
    -> 航点 11/12/13: 限时识别目标颜色
       -> 识别成功 -> 飞往投放点 -> 释放 -> 降落
       -> 全部失败 -> 飞回备降航点 -> 释放 -> 降落

用法:
    python3 opencv_nav_micro.py
"""
import os
import sys
import signal
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State, BatteryStatus
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf import transformations
import cv2
import numpy as np
import time
import serial
import threading


# ======================== 视觉识别类 ========================

class RobotVision:
    """基于 OpenCV HSV 的颜色圆形目标识别"""

    # 默认 HSV 阈值 (可通过 ROS 参数覆盖)
    DEFAULT_COLOR_RANGES = {
        'Red': [
            {'lower': [0, 150, 70], 'upper': [10, 255, 255]},
            {'lower': [170, 150, 70], 'upper': [180, 255, 255]},
        ],
        'Green': [
            {'lower': [35, 100, 50], 'upper': [85, 255, 255]},
        ],
        'Blue': [
            {'lower': [100, 130, 50], 'upper': [130, 255, 255]},
        ],
    }

    def __init__(self, color_ranges=None, min_area=1000, min_circularity=0.5, process_width=640):
        self.color_ranges = color_ranges or self.DEFAULT_COLOR_RANGES
        # 将列表转为 numpy 数组 (只做一次)
        self._np_ranges = {}
        for color_name, masks in self.color_ranges.items():
            self._np_ranges[color_name] = [
                {
                    'lower': np.array(m['lower'], dtype=np.uint8),
                    'upper': np.array(m['upper'], dtype=np.uint8),
                }
                for m in masks
            ]
        self.min_area = min_area
        self.min_circularity = min_circularity
        self.process_width = process_width
        self._morph_kernel = np.ones((5, 5), np.uint8)

    def detect_target(self, frame, target_color=None):
        """
        核心识别函数

        Args:
            frame: BGR 图像帧
            target_color: 若指定，仅检测该颜色 (如 'Red')

        Returns:
            (detected_name, center, annotated_frame)
        """
        # 缩放到处理尺寸以保障实时性
        if frame.shape[1] > self.process_width:
            scale = self.process_width / frame.shape[1]
            frame = cv2.resize(frame, (self.process_width, int(frame.shape[0] * scale)))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_name = "None"
        center = None
        img_h, img_w = frame.shape[:2]

        # 选择搜索范围
        if target_color and target_color in self._np_ranges:
            search_colors = {target_color: self._np_ranges[target_color]}
        else:
            search_colors = self._np_ranges

        for color_name, masks in search_colors.items():
            full_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for m in masks:
                full_mask |= cv2.inRange(hsv, m['lower'], m['upper'])

            # 闭运算闭合圆环空洞
            full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_CLOSE, self._morph_kernel)

            contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue

                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue

                circularity = (4 * np.pi * area) / (perimeter ** 2)
                if circularity < self.min_circularity:
                    continue

                (x, y), radius = cv2.minEnclosingCircle(cnt)

                # 检查目标是否完整 (不在画面边缘)
                if (x - radius > 2 and x + radius < img_w - 2 and
                        y - radius > 2 and y + radius < img_h - 2):
                    center = (int(x), int(y))
                    detected_name = color_name

                    # 绘制标注
                    cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                    cv2.circle(frame, center, 3, (0, 0, 255), -1)
                    cv2.putText(frame, color_name,
                                (center[0] - 20, center[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    break

            if center:
                break

        return detected_name, center, frame


# ======================== 串口爪子控制类 ========================

class ClawController:
    """ESP8266 串口舵机爪控制器"""

    GRAB_CMD = b'1'
    RELEASE_CMD = b'2'

    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self._read_thread = None
        self._stop_event = threading.Event()
        self.connect()

    def connect(self):
        """连接串口并初始化"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            # 防止 DTR/RTS 信号卡死板子
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            time.sleep(1)
            # 复位脉冲
            self.ser.setDTR(True)
            time.sleep(0.1)
            self.ser.setDTR(False)

            rospy.loginfo("爪子串口已连接: %s @ %d", self.port, self.baud)
            rospy.loginfo("等待爪子硬件初始化 (3s)...")
            time.sleep(3)

            # 启动后台监听线程
            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
        except serial.SerialException as e:
            rospy.logerr("爪子串口连接失败: %s", e)
        except OSError as e:
            rospy.logerr("串口设备不存在或权限不足: %s", e)

    def _read_loop(self):
        """后台监听 ESP8266 反馈"""
        while not self._stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        rospy.loginfo("[爪子反馈] %s", line)
            except serial.SerialException:
                rospy.logwarn("爪子串口读取异常，监听线程退出。")
                break
            except UnicodeDecodeError:
                continue  # 忽略解码错误
            self._stop_event.wait(0.01)

    def grab(self):
        """控制爪子抓取"""
        rospy.loginfo("控制爪子: 抓取")
        self._send_cmd(self.GRAB_CMD)

    def release(self):
        """控制爪子张开"""
        rospy.loginfo("控制爪子: 张开")
        self._send_cmd(self.RELEASE_CMD)

    def _send_cmd(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd)
            except serial.SerialException as e:
                rospy.logerr("爪子串口写入失败: %s", e)
        else:
            rospy.logwarn("爪子串口未连接，指令未发送。")

    def close(self):
        """关闭串口连接"""
        self._stop_event.set()
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("爪子串口已关闭。")


# ======================== 导航控制器类 ========================

class NavigationController:
    """无人机自主导航控制器 (含视觉识别任务的完整版)"""

    def __init__(self):
        # ---- ROS 参数 ----
        self.target_z = rospy.get_param('~target_z', 0.0)
        self.kp_z = rospy.get_param('~kp_z', 1.5)
        self.kd_z = rospy.get_param('~kd_z', 0.0)
        self.max_xy_speed = rospy.get_param('~max_xy_speed', 1.5)
        self.max_z_speed = rospy.get_param('~max_z_speed', 0.8)
        self.waypoint_xy_tol = rospy.get_param('~waypoint_xy_tol', 0.3)
        self.waypoint_z_tol = rospy.get_param('~waypoint_z_tol', 0.15)
        self.waypoint_timeout = rospy.get_param('~waypoint_timeout', 120.0)
        self.direct_flight_timeout = rospy.get_param('~direct_flight_timeout', 60.0)
        self.takeoff_timeout = rospy.get_param('~takeoff_timeout', 30.0)
        self.land_timeout = rospy.get_param('~land_timeout', 60.0)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)
        self.takeoff_height = rospy.get_param('~takeoff_height', 0.7)

        # 视觉任务参数 (可通过 ROS 参数覆盖)
        self.vision_detect_wp = rospy.get_param('~vision_detect_wp', 4)      # 识别航点序号
        self.vision_grab_wp = rospy.get_param('~vision_grab_wp', 5)          # 抓取航点序号
        self.vision_search_wps = rospy.get_param('~vision_search_wps', [11, 12, 13])  # 搜索航点
        self.vision_timeout_sec = rospy.get_param('~vision_timeout_sec', 6.0)  # 视觉识别超时
        self.drop_offset_x = rospy.get_param('~drop_offset_x', 0.0)
        self.drop_offset_y = rospy.get_param('~drop_offset_y', 0.0)
        self.drop_offset_z = rospy.get_param('~drop_offset_z', 0.0)
        self.fallback_land_wp_index = rospy.get_param('~fallback_land_wp_index', 11)
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.camera_width = rospy.get_param('~camera_width', 1920)
        self.camera_height = rospy.get_param('~camera_height', 1080)
        self.claw_port = rospy.get_param('~claw_port', '/dev/ttyUSB0')
        self.claw_baud = rospy.get_param('~claw_baud', 115200)

        # ---- 内部状态 ----
        self.current_state = State()
        self.current_position = PoseStamped()
        self.battery = BatteryStatus()
        self.pose_received = False
        self.last_pose_time = rospy.Time(0)
        self.now_yaw = 0.0
        self.cmd_vel_data = Twist()
        self.rate = rospy.Rate(20)
        self._prev_err_z = 0.0
        self._emergency_land_triggered = False
        self._home_position = None

        # ---- 发布者 / 订阅者 / 服务 ----
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/battery', BatteryStatus, self._battery_cb)

        rospy.wait_for_service('/mavros/set_mode', timeout=30)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        try:
            rospy.wait_for_service('/mavros/cmd/land', timeout=5)
            self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        except rospy.ROSException:
            self.land_client = None

        self.waypoint_file = self._get_waypoint_path()

        # ---- 信号处理 ----
        signal.signal(signal.SIGINT, self._sigint_handler)

    # ---------- 回调 ----------

    def _state_cb(self, msg):
        self.current_state = msg
        if not msg.connected and self.pose_received:
            rospy.logerr("MAVROS 连接断开!")
            self._trigger_emergency_land()

    def _battery_cb(self, msg):
        self.battery = msg
        if 0 <= msg.percentage < self.low_battery_threshold and self.pose_received:
            rospy.logerr("低电量: %.1f%% (阈值 %.1f%%)", msg.percentage, self.low_battery_threshold)
            self._trigger_emergency_land()

    def _cmd_vel_cb(self, msg):
        self.cmd_vel_data.linear.x = self._clamp(msg.linear.x, -self.max_xy_speed, self.max_xy_speed)
        self.cmd_vel_data.linear.y = self._clamp(msg.linear.y, -self.max_xy_speed, self.max_xy_speed)
        self.cmd_vel_data.angular.z = msg.angular.z
        if abs(msg.linear.x) > self.max_xy_speed or abs(msg.linear.y) > self.max_xy_speed:
            rospy.logwarn_throttle(1.0, "cmd_vel 限幅")

    def _pose_cb(self, msg):
        self.current_position = msg
        self.pose_received = True
        self.last_pose_time = rospy.Time.now()
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        self.now_yaw = transformations.euler_from_quaternion(q)[2]

    # ---------- 工具 ----------

    def _get_waypoint_path(self):
        ros_param = rospy.get_param('~waypoint_file', '')
        if ros_param and os.path.isfile(ros_param):
            return ros_param
        env_path = os.environ.get('DRONE_WAYPOINT_FILE', '')
        if env_path and os.path.isfile(env_path):
            return env_path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        for candidate in [
            os.path.join(script_dir, 'point.txt'),
            os.path.expanduser('~/test_points.txt'),
            os.path.expanduser('~/point.txt'),
        ]:
            if os.path.isfile(candidate):
                return candidate
        return os.path.join(script_dir, 'point.txt')

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _pose_is_fresh(self, max_age=1.0):
        return self.pose_received and (rospy.Time.now() - self.last_pose_time).to_sec() <= max_age

    def _check_timeout(self, start, timeout):
        return (rospy.Time.now() - start).to_sec() > timeout

    def get_distance_to_target(self, tx, ty):
        dx = self.current_position.pose.position.x - tx
        dy = self.current_position.pose.position.y - ty
        return math.hypot(dx, dy)

    @staticmethod
    def load_waypoints(filepath):
        waypoints = []
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                s = line.strip()
                if not s or s.startswith('#'):
                    continue
                parts = s.split()
                if len(parts) >= 4:
                    try:
                        waypoints.append((float(parts[0]), float(parts[1]),
                                          float(parts[2]), float(parts[3])))
                    except ValueError:
                        rospy.logwarn("航点文件第 %d 行解析失败: %s", line_num, s)
                else:
                    rospy.logwarn("航点文件第 %d 行格式无效: %s", line_num, s)
        return waypoints

    # ---------- 安全保护 ----------

    def _trigger_emergency_land(self):
        if self._emergency_land_triggered:
            return
        self._emergency_land_triggered = True
        rospy.logerr("!!! 紧急降落 !!!")
        self.land_at_current_position()

    def _check_emergency(self):
        if self._emergency_land_triggered:
            return True
        if self.pose_received and not self._pose_is_fresh(3.0):
            rospy.logerr("位姿数据超时 (>3s)")
            self._trigger_emergency_land()
            return True
        return False

    def _sigint_handler(self, signum, frame):
        rospy.loginfo("收到中断信号，安全降落...")
        if self.pose_received and self.current_position.pose.position.z > 0.15:
            self.land_at_current_position()
        sys.exit(0)

    # ---------- 飞行控制 ----------

    def send_velocity_setpoint(self, vx_body, vy_body, vz, yaw_rate):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = 1479
        cy, sy = math.cos(self.now_yaw), math.sin(self.now_yaw)
        sp.velocity.x = vx_body * cy - vy_body * sy
        sp.velocity.y = vx_body * sy + vy_body * cy
        sp.velocity.z = self._clamp(vz, -self.max_z_speed, self.max_z_speed)
        sp.yaw_rate = yaw_rate
        self.setpoint_pub.publish(sp)

    def send_position_setpoint(self, x, y, z, yaw=None):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = 2552
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.yaw = yaw if yaw is not None else self.now_yaw
        self.setpoint_pub.publish(sp)

    def _switch_mode(self, mode):
        try:
            resp = self.set_mode_client(custom_mode=mode)
            if resp.mode_sent:
                rospy.loginfo("模式切换成功: %s", mode)
            else:
                rospy.logerr("模式切换失败: %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("模式切换异常 (%s): %s", mode, e)

    # ---------- 模式与解锁 ----------

    def wait_for_fcu_ready(self, timeout=30.0):
        rospy.loginfo("等待 MAVROS 连接和本地位姿...")
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.connected and self._pose_is_fresh():
                rospy.loginfo("MAVROS 已连接，本地位姿可用。")
                return True
            if self._check_timeout(start, timeout):
                rospy.logerr("等待 MAVROS/位姿超时。")
                return False
            rospy.loginfo_throttle(2.0, "等待中... connected=%s pose=%s",
                                   self.current_state.connected, self.pose_received)
            self.rate.sleep()
        return False

    def set_offboard_and_arm(self):
        if not self.wait_for_fcu_ready():
            return False
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        rospy.loginfo("预发布设定点 (5s)...")
        for _ in range(100):
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)
            self.rate.sleep()
        rospy.loginfo("设置 OFFBOARD 模式...")
        try:
            resp = self.set_mode_client(custom_mode='OFFBOARD')
            if not resp.mode_sent:
                rospy.logerr("OFFBOARD 模式设置失败!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("OFFBOARD 设置异常: %s", e)
            return False
        rospy.loginfo("解锁飞行器...")
        try:
            resp = self.arm_client(True)
            if not resp.success:
                rospy.logerr("解锁失败!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("解锁异常: %s", e)
            return False
        return True

    # ---------- 飞行任务 ----------

    def takeoff(self, height=None):
        self.target_z = height if height is not None else self.takeoff_height
        rospy.loginfo("起飞至 %.2f 米...", self.target_z)
        if not self.set_offboard_and_arm():
            rospy.logerr("起飞中止。")
            return False
        self._home_position = (
            self.current_position.pose.position.x,
            self.current_position.pose.position.y,
        )
        lock_x, lock_y, lock_yaw = (
            self.current_position.pose.position.x,
            self.current_position.pose.position.y,
            self.now_yaw,
        )
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)
            if self._check_emergency():
                return False
            cz = self.current_position.pose.position.z
            if abs(cz - self.target_z) < self.waypoint_z_tol:
                rospy.loginfo("到达目标高度: %.2f 米", cz)
                return True
            if self._check_timeout(start, self.takeoff_timeout):
                rospy.logerr("起飞超时。当前: %.2f", cz)
                return False
            rospy.loginfo_throttle(2.0, "上升中... %.2f / %.2f", cz, self.target_z)
            self.rate.sleep()
        return False

    def navigation_target(self, x, y, z, hover_time=2.0):
        self.target_z = z
        rospy.sleep(1.0)
        self.get_goal(x, y)
        rospy.loginfo("导航至 x=%.2f, y=%.2f, z=%.2f", x, y, z)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._check_emergency():
                return False
            err_z = self.target_z - self.current_position.pose.position.z
            d_err_z = (err_z - self._prev_err_z) * 20.0
            v_z_pid = self.kp_z * err_z + self.kd_z * d_err_z
            self._prev_err_z = err_z
            self.send_velocity_setpoint(
                self.cmd_vel_data.linear.x, self.cmd_vel_data.linear.y,
                v_z_pid, yaw_rate=0.0,
            )
            if self.get_distance_to_target(x, y) < self.waypoint_xy_tol:
                rospy.loginfo("到达航点!")
                break
            if self._check_timeout(start, self.waypoint_timeout):
                rospy.logwarn("航点导航超时，跳转。")
                break
            self.rate.sleep()
        rospy.loginfo("悬停 %.1fs...", hover_time)
        hs = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - hs).to_sec() < hover_time:
            self.send_position_setpoint(x, y, z)
            self.rate.sleep()

    def fly_to_point_directly(self, x, y, z, tol_xy=0.3, tol_z=0.2, timeout=None):
        timeout = self.direct_flight_timeout if timeout is None else timeout
        rospy.loginfo("直接飞行至 x=%.2f, y=%.2f, z=%.2f", x, y, z)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            self.send_position_setpoint(x, y, z)
            dx = self.current_position.pose.position.x - x
            dy = self.current_position.pose.position.y - y
            dz = self.current_position.pose.position.z - z
            if math.hypot(dx, dy) < tol_xy and abs(dz) < tol_z:
                rospy.loginfo("到达直接飞行目标!")
                return True
            if self._check_timeout(start, timeout):
                rospy.logwarn("直接飞行超时。")
                return False
            self.rate.sleep()
        return False

    def land_at_current_position(self):
        rospy.loginfo("安全降落...")
        if not self.pose_received:
            self._switch_mode('AUTO.LAND')
            return
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        cz = self.current_position.pose.position.z
        safe_z = 0.15
        if cz > safe_z:
            rospy.loginfo("缓慢垂直降落...")
            target_z = cz
            start = rospy.Time.now()
            while not rospy.is_shutdown() and cz > safe_z + 0.05:
                target_z = max(target_z - 0.02, safe_z)
                self.send_position_setpoint(lock_x, lock_y, target_z, yaw=lock_yaw)
                cz = self.current_position.pose.position.z
                self.rate.sleep()
                if self._check_timeout(start, self.land_timeout):
                    break
        self._switch_mode('AUTO.LAND')

    def get_goal(self, x, y):
        g = PoseStamped()
        g.header.stamp = rospy.Time.now()
        g.header.frame_id = "map"
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    # ---------- 视觉任务辅助 ----------

    def _open_camera(self):
        """打开摄像头并配置参数"""
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FPS, 30)
        if not cap.isOpened():
            rospy.logerr("摄像头 %d 打开失败!", self.camera_id)
            return None
        return cap

    @staticmethod
    def _close_camera(cap):
        """安全关闭摄像头"""
        if cap and cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()


# ======================== 主逻辑 ========================

def main():
    rospy.init_node('navigation_controller', anonymous=True)
    nav = NavigationController()
    vision = RobotVision()
    claw = ClawController(port=nav.claw_port, baud=nav.claw_baud)

    COLOR_CN = {'Red': '红色', 'Green': '绿色', 'Blue': '蓝色'}

    rospy.sleep(2.0)

    # 1. 起飞
    if not nav.takeoff():
        rospy.logerr("起飞失败，跳过导航。")
        claw.close()
        return

    # 2. 读取航点
    try:
        waypoints = NavigationController.load_waypoints(nav.waypoint_file)
    except IOError:
        rospy.logerr("航点文件打开失败: %s", nav.waypoint_file)
        nav.land_at_current_position()
        claw.close()
        return

    if not waypoints:
        rospy.logwarn("航点文件为空，悬停后降落。")
        rospy.sleep(3.0)
        nav.land_at_current_position()
        claw.close()
        return

    rospy.loginfo("共 %d 个航点，开始执行...", len(waypoints))

    remembered_color = None
    early_land = False

    # 3. 遍历航点
    for i, (x, y, z, t) in enumerate(waypoints):
        if early_land or nav._emergency_land_triggered:
            break

        wp_idx = i + 1
        rospy.loginfo("=" * 50 + " 航点 #%d/%d " + "=" * 50, wp_idx, len(waypoints))
        nav.navigation_target(x, y, z, t)

        # ---- 逻辑1: 视觉识别航点 (默认第4个) ----
        if wp_idx == nav.vision_detect_wp:
            rospy.loginfo(">>> 航点 %d: 第一次视觉识别...", wp_idx)
            cap = nav._open_camera()
            if cap is None:
                continue

            start = time.time()
            while not rospy.is_shutdown() and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break
                color, pos, _ = vision.detect_target(frame)
                if color != "None" and pos is not None:
                    remembered_color = color
                    rospy.loginfo("********** 锁定颜色: 【%s】 **********",
                                  COLOR_CN.get(color, color))
                    break
                if time.time() - start > nav.vision_timeout_sec:
                    rospy.logwarn("第一次视觉识别超时。")
                    break

            nav._close_camera(cap)

        # ---- 逻辑2: 抓取航点 (默认第5个) ----
        elif wp_idx == nav.vision_grab_wp:
            rospy.loginfo(">>> 航点 %d: 准备抓取...", wp_idx)
            rospy.loginfo("抓取前悬停 3 秒...")
            hs = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - hs).to_sec() < 3.0:
                nav.send_position_setpoint(x, y, z)
                nav.rate.sleep()
            claw.grab()
            rospy.loginfo("已抓取! 继续带球导航...")

        # ---- 逻辑3: 搜索投放航点 (默认 11/12/13) ----
        elif wp_idx in nav.vision_search_wps and remembered_color is not None:
            rospy.loginfo(">>> 航点 %d: 限时视觉识别...", wp_idx)
            cap = nav._open_camera()
            if cap is None:
                continue

            detected = False
            start = time.time()
            while not rospy.is_shutdown() and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break
                color, pos, _ = vision.detect_target(frame, target_color=remembered_color)
                if color != "None" and pos is not None:
                    detected = True
                    rospy.loginfo("********** 航点 %d 识别到: 【%s】 **********",
                                  wp_idx, COLOR_CN.get(color, color))
                    break
                if time.time() - start > nav.vision_timeout_sec:
                    rospy.loginfo("航点 %d 识别超时，未发现目标。", wp_idx)
                    break

            nav._close_camera(cap)

            if detected:
                # 飞往投放点
                land_x = nav.current_position.pose.position.x + nav.drop_offset_x
                land_y = nav.current_position.pose.position.y + nav.drop_offset_y
                land_z = nav.current_position.pose.position.z + nav.drop_offset_z
                rospy.loginfo("飞往投放点: x=%.2f, y=%.2f, z=%.2f", land_x, land_y, land_z)
                nav.fly_to_point_directly(land_x, land_y, land_z, tol_xy=0.2, tol_z=0.15)

                rospy.loginfo("投放点悬停 2 秒...")
                hs = rospy.Time.now()
                while not rospy.is_shutdown() and (rospy.Time.now() - hs).to_sec() < 2.0:
                    nav.send_position_setpoint(land_x, land_y, land_z)
                    nav.rate.sleep()

                claw.release()
                rospy.loginfo("小球已投放!")
                nav.land_at_current_position()
                early_land = True

            # 最后一个搜索航点仍未识别到
            elif wp_idx == nav.vision_search_wps[-1]:
                rospy.logwarn("所有搜索航点均未识别到目标，飞回备降航点 %d...",
                              nav.fallback_land_wp_index)
                idx = nav.fallback_land_wp_index
                if 0 < idx <= len(waypoints):
                    fb_x, fb_y, fb_z, _ = waypoints[idx - 1]
                    nav.fly_to_point_directly(fb_x, fb_y, fb_z)
                    hs = rospy.Time.now()
                    while not rospy.is_shutdown() and (rospy.Time.now() - hs).to_sec() < 2.0:
                        nav.send_position_setpoint(fb_x, fb_y, fb_z)
                        nav.rate.sleep()
                    claw.release()
                    rospy.loginfo("小球已投放!")
                    nav.land_at_current_position()
                    early_land = True
                else:
                    rospy.logerr("备降航点序号 %d 超出范围，原地降落!", idx)
                    nav.land_at_current_position()
                    early_land = True

    # 4. 所有航点完成后降落
    if not early_land and not nav._emergency_land_triggered:
        rospy.loginfo("所有航点完成! 悬停 2 秒后降落...")
        hs = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - hs).to_sec() < 2.0:
            nav.send_position_setpoint(waypoints[-1][0], waypoints[-1][1], waypoints[-1][2])
            nav.rate.sleep()
        claw.release()
        nav.land_at_current_position()

    rospy.loginfo("任务结束!")
    claw.close()


if __name__ == "__main__":
    main()
