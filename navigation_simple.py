#!/usr/bin/env python3
import os
import sys
import math
import signal
import threading
import traceback
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State, BatteryStatus
from mavros_msgs.srv import CommandBool, SetMode
from actionlib_msgs.msg import GoalID
from tf import transformations

# ===== type_mask 位定义 (置 1 = 忽略该字段, 与 MAVLink/mavros_msgs 一致) =====
IGNORE_X, IGNORE_Y, IGNORE_Z = 1, 2, 4
IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
IGNORE_AX, IGNORE_AY, IGNORE_AZ = 64, 128, 256
IGNORE_YAW, IGNORE_YAW_RATE = 512, 1024

# 速度 + 偏航角速率 (= 967; 旧值 1479 实际是"速度+偏航角", yaw_rate 被忽略)
MASK_VEL_YAW_RATE = (IGNORE_X | IGNORE_Y | IGNORE_Z |
                     IGNORE_AX | IGNORE_AY | IGNORE_AZ |
                     IGNORE_YAW)
# 位置 + 偏航角 (= 1528; 旧值 2552 多设了 FORCE 位)
MASK_POS_YAW = (IGNORE_VX | IGNORE_VY | IGNORE_VZ |
                IGNORE_AX | IGNORE_AY | IGNORE_AZ |
                IGNORE_YAW_RATE)


class NavigationController:

    def __init__(self):
        # ---- ROS 参数 ----
        self.takeoff_height = rospy.get_param('~takeoff_height', 0.8)
        self.kp_z = rospy.get_param('~kp_z', 1.5)
        self.kd_z = rospy.get_param('~kd_z', 0.0)
        self.max_xy_speed = rospy.get_param('~max_xy_speed', 1.5)
        self.max_z_speed = rospy.get_param('~max_z_speed', 0.8)
        self.waypoint_xy_tol = rospy.get_param('~waypoint_xy_tol', 0.3)
        self.waypoint_z_tol = rospy.get_param('~waypoint_z_tol', 0.15)
        self.waypoint_timeout = rospy.get_param('~waypoint_timeout', 120.0)
        self.takeoff_timeout = rospy.get_param('~takeoff_timeout', 30.0)
        self.land_timeout = rospy.get_param('~land_timeout', 60.0)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)  # %
        self.low_battery_action = rospy.get_param('~low_battery_action', 'land')
        self.goal_frame = rospy.get_param('~goal_frame', 'map')
        self.cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout', 0.5)
        self.goal_connect_timeout = rospy.get_param('~goal_connect_timeout', 5.0)

        # ---- 内部状态 ----
        self.current_state = State()
        self.current_position = PoseStamped()
        self.battery = BatteryStatus()
        self.battery_received = False
        self.pose_received = False
        self.last_pose_time = rospy.Time(0)
        self.now_yaw = 0.0
        self.cmd_vel_data = Twist()
        self.cmd_vel_time = rospy.Time(0)
        self.rate = rospy.Rate(20)
        self.target_z = 0.0
        self._prev_err_z = 0.0
        self._last_pid_time = rospy.Time(0)

        self._emergency_lock = threading.Lock()
        self._emergency_land_triggered = False
        self._emergency_reason = ''
        self._shutdown_requested = False

        # ---- 发布者 ----
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # ---- 订阅者 (回调只做轻量记录/置标志, 绝不跑降落流程) ----
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.current_position_callback)
        rospy.Subscriber('/mavros/battery', BatteryStatus, self.battery_callback)

        # ---- 服务 ----
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=30)
            self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
            self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logfatal("等待 MAVROS 服务超时, 请确认 mavros 已启动")
            sys.exit(1)

        self.waypoint_file = self._get_waypoint_path()
        signal.signal(signal.SIGINT, self._sigint_handler)

    # ===================== 公开状态 =====================

    @property
    def emergency(self):
        return self._emergency_land_triggered

    @property
    def shutdown_requested(self):
        return self._shutdown_requested

    @property
    def airborne(self):
        return self.pose_received and self.current_position.pose.position.z > 0.15

    # ===================== 回调 =====================

    def state_callback(self, msg):
        was_armed = self.current_state.armed
        self.current_state = msg
        if not msg.connected and self.pose_received and was_armed:
            self._trigger_emergency_land('disconnect', 'MAVROS 连接断开')
        if was_armed and not msg.armed and self.airborne:
            self._trigger_emergency_land('disarm', '飞行中意外上锁 (飞控 failsafe?)')

    def battery_callback(self, msg):
        self.battery = msg
        self.battery_received = True
        # 注意: ROS1 MAVROS 的 percentage 是 0~1; -1/NaN 表示未知
        p = msg.percentage
        if math.isnan(p) or p < 0.0 or p > 1.0:
            return
        if p * 100.0 < self.low_battery_threshold and self.current_state.armed and self.airborne:
            self._trigger_emergency_land('low_battery', '剩余电量 %.0f%%' % (p * 100.0))

    def cmd_vel_callback(self, msg):
        # 真正的限幅 (旧版是丢弃整条消息并沿用旧值)
        self.cmd_vel_data.linear.x = self._clamp(msg.linear.x, -self.max_xy_speed, self.max_xy_speed)
        self.cmd_vel_data.linear.y = self._clamp(msg.linear.y, -self.max_xy_speed, self.max_xy_speed)
        if abs(msg.linear.x) > self.max_xy_speed or abs(msg.linear.y) > self.max_xy_speed:
            rospy.logwarn_throttle(1.0, "cmd_vel 限幅: vx=%.2f vy=%.2f", msg.linear.x, msg.linear.y)
        self.cmd_vel_time = rospy.Time.now()
        # angular.z 不使用: 导航时锁定偏航 (yaw_rate=0)

    def current_position_callback(self, msg):
        self.current_position = msg
        self.pose_received = True
        self.last_pose_time = rospy.Time.now()
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        self.now_yaw = transformations.euler_from_quaternion(q)[2]

    # ===================== 工具 =====================

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def _pose_is_fresh(self, max_age=1.0):
        if not self.pose_received:
            return False
        return (rospy.Time.now() - self.last_pose_time).to_sec() <= max_age

    def _check_timeout(self, start_time, timeout):
        return (rospy.Time.now() - start_time).to_sec() > timeout

    def get_distance_to_target(self, target_x, target_y):
        dx = self.current_position.pose.position.x - target_x
        dy = self.current_position.pose.position.y - target_y
        return math.hypot(dx, dy)

    def _get_waypoint_path(self):
        ros_param_path = rospy.get_param('~waypoint_file', '')
        if ros_param_path and os.path.isfile(ros_param_path):
            return ros_param_path
        env_path = os.environ.get('DRONE_WAYPOINT_FILE', '')
        if env_path and os.path.isfile(env_path):
            return env_path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [os.path.join(script_dir, 'point.txt'),
                      os.path.expanduser('~/point.txt')]
        for path in candidates:
            if os.path.isfile(path):
                return path
        rospy.logwarn("未找到航点文件, 使用默认路径: %s", candidates[0])
        return candidates[0]

    @staticmethod
    def load_waypoints(filepath):
        """解析航点文件: x y z hover_time; 支持 # 注释/空行, 坏行不崩溃"""
        waypoints = []
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                stripped = line.strip()
                if not stripped or stripped.startswith('#'):
                    continue
                parts = stripped.split()
                if len(parts) >= 4:
                    try:
                        x, y, z, t = (float(parts[0]), float(parts[1]),
                                      float(parts[2]), float(parts[3]))
                    except ValueError:
                        rospy.logwarn("航点第 %d 行解析失败: %s", line_num, stripped)
                        continue
                    if z < 0.15:
                        rospy.logwarn("航点第 %d 行高度 %.2f 过低, 提高到 0.3", line_num, z)
                        z = 0.3
                    waypoints.append((x, y, z, t))
                else:
                    rospy.logwarn("航点第 %d 行格式无效 (x y z hover_time): %s",
                                  line_num, stripped)
        return waypoints

    # ===================== 紧急与中断 =====================

    def _trigger_emergency_land(self, reason_code, detail=''):
        """只置标志 (线程安全); 降落统一由主线程执行, 回调里绝不跑降落流程"""
        with self._emergency_lock:
            if self._emergency_land_triggered:
                return
            self._emergency_land_triggered = True
            self._emergency_reason = reason_code
        rospy.logerr("=" * 40)
        rospy.logerr("!!! 紧急降落触发 [%s]: %s !!!", reason_code, detail)
        rospy.logerr("=" * 40)

    def _check_emergency(self):
        if self._emergency_land_triggered:
            return True
        if self.pose_received and not self._pose_is_fresh(max_age=3.0):
            self._trigger_emergency_land('pose_timeout', '位姿 >3s 无更新')
            return True
        if not self.current_state.connected and self.pose_received and self.airborne:
            self._trigger_emergency_land('disconnect', 'MAVROS 连接断开')
            return True
        return False

    def _should_abort(self):
        return self._shutdown_requested or self._check_emergency()

    def _sigint_handler(self, signum, frame):
        if self._shutdown_requested:
            rospy.logerr("再次中断, 强制退出。")
            sys.exit(1)
        rospy.logwarn("收到中断: 将安全降落后退出 (再按一次 Ctrl+C 强制退出)")
        self._shutdown_requested = True

    # ===================== 设定点发送 =====================

    def send_velocity_setpoint(self, v_x_body, v_y_body, v_z, yaw_rate=0.0):
        """机体系(前/左) → ENU; yaw_rate=0 锁定当前航向"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = MASK_VEL_YAW_RATE
        cos_yaw = math.cos(self.now_yaw)
        sin_yaw = math.sin(self.now_yaw)
        setpoint.velocity.x = v_x_body * cos_yaw - v_y_body * sin_yaw
        setpoint.velocity.y = v_x_body * sin_yaw + v_y_body * cos_yaw
        setpoint.velocity.z = self._clamp(v_z, -self.max_z_speed, self.max_z_speed)
        setpoint.yaw_rate = yaw_rate
        self.setpoint_pub.publish(setpoint)

    def send_position_setpoint(self, x, y, z, yaw=None):
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = MASK_POS_YAW
        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.yaw = self.now_yaw if yaw is None else yaw
        self.setpoint_pub.publish(setpoint)

    def _hold_at(self, x, y, z, seconds):
        """定点悬停 —— 期间持续发送设定点, 保证 OFFBOARD 不断流"""
        end_time = rospy.Time.now() + rospy.Duration(seconds)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            if self._should_abort():
                return False
            self.send_position_setpoint(x, y, z)
            self.rate.sleep()
        return True

    def hold_position(self, seconds):
        return self._hold_at(self.current_position.pose.position.x,
                             self.current_position.pose.position.y,
                             self.current_position.pose.position.z,
                             seconds)

    # ===================== 模式与解锁 =====================

    def wait_for_fcu_ready(self, timeout=30.0):
        rospy.loginfo("等待 MAVROS 连接和本地位姿...")
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not self._shutdown_requested:
            if self.current_state.connected and self._pose_is_fresh():
                rospy.loginfo("MAVROS 已连接, 本地位姿可用。")
                return True
            if self._check_timeout(start_time, timeout):
                rospy.logerr("等待 MAVROS/位姿超时 (%.0fs)。", timeout)
                return False
            rospy.loginfo_throttle(2.0, "等待中... connected=%s pose_received=%s",
                                   self.current_state.connected, self.pose_received)
            self.rate.sleep()
        return False

    def _preflight_battery_ok(self):
        if not self.battery_received:
            rospy.logwarn("未收到电池数据, 跳过低电量检查 (检查 /mavros/battery?)")
            return True
        p = self.battery.percentage
        if math.isnan(p) or p < 0.0 or p > 1.0:
            rospy.logwarn("电池数据无效 (%.2f), 跳过低电量检查", p)
            return True
        if p * 100.0 < self.low_battery_threshold:
            rospy.logerr("电量 %.0f%% 低于阈值 %.0f%%, 拒绝起飞!",
                         p * 100.0, self.low_battery_threshold)
            return False
        rospy.loginfo("当前电量: %.0f%%", p * 100.0)
        return True

    def set_offboard_and_arm(self):
        if not self.wait_for_fcu_ready():
            return False
        if not self._preflight_battery_ok():
            return False

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        rospy.loginfo("预发布设定点 (5s)...")
        for _ in range(100):
            if self._should_abort():
                return False
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)
            self.rate.sleep()

        rospy.loginfo("切换 OFFBOARD 模式...")
        try:
            resp = self.set_mode_client(custom_mode='OFFBOARD')
            if not resp.mode_sent:
                rospy.logerr("设置 OFFBOARD 失败!")
                return False
            rospy.loginfo("OFFBOARD 已启用。")
        except rospy.ServiceException as e:
            rospy.logerr("设置 OFFBOARD 异常: %s", e)
            return False

        rospy.loginfo("解锁...")
        try:
            resp = self.arm_client(True)
            if not resp.success:
                rospy.logerr("解锁失败!")
                return False
            rospy.loginfo("已解锁。")
        except rospy.ServiceException as e:
            rospy.logerr("解锁异常: %s", e)
            return False
        return True

    # ===================== 飞行任务 =====================

    def takeoff(self, height=None):
        self.target_z = self.takeoff_height if height is None else height
        rospy.loginfo("起飞至 %.2f 米...", self.target_z)

        if not self.set_offboard_and_arm():
            rospy.logerr("起飞中止: 模式/解锁失败。")
            return False

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)
            if self._should_abort():
                return False
            current_z = self.current_position.pose.position.z
            if abs(current_z - self.target_z) < self.waypoint_z_tol:
                rospy.loginfo("到达目标高度: %.2f 米", current_z)
                return True
            if self._check_timeout(start_time, self.takeoff_timeout):
                rospy.logerr("起飞超时, 当前高度 %.2f 米", current_z)
                return False
            rospy.loginfo_throttle(2.0, "上升中... %.2f / %.2f", current_z, self.target_z)
            self.rate.sleep()
        return False

    def navigation_target(self, x, y, z, hover_time=2.0):
        """XY 由 move_base 规划, Z 由 PD 控制。返回是否到达"""
        self.target_z = z
        # 复位 PD 状态 (避免上一航点残差造成 D 尖峰); 清零残余 cmd_vel
        self._prev_err_z = z - self.current_position.pose.position.z
        self._last_pid_time = rospy.Time.now()
        self.cmd_vel_data = Twist()
        self.cmd_vel_time = rospy.Time(0)

        arrived = False
        try:
            # 等待 move_base 订阅连接 —— 期间持续发设定点, 不断流
            hold_x = self.current_position.pose.position.x
            hold_y = self.current_position.pose.position.y
            hold_z = self.current_position.pose.position.z
            start_time = rospy.Time.now()
            while self.goal_pub.get_num_connections() == 0:
                if self._should_abort():
                    return False
                if self._check_timeout(start_time, self.goal_connect_timeout):
                    rospy.logerr("move_base 未连接, 跳过本航点")
                    self._hold_at(hold_x, hold_y, hold_z, 2.0)
                    return False
                self.send_position_setpoint(hold_x, hold_y, hold_z)
                self.rate.sleep()

            self.get_goal(x, y)
            rospy.loginfo("导航至 (%.2f, %.2f, %.2f), 悬停 %.1fs", x, y, z, hover_time)

            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self._should_abort():
                    return False

                # 高度 PD (实测 dt, dt 异常时跳过 D 项)
                now = rospy.Time.now()
                dt = (now - self._last_pid_time).to_sec()
                err_z = self.target_z - self.current_position.pose.position.z
                if 0.005 < dt < 0.5:
                    d_err_z = (err_z - self._prev_err_z) / dt
                else:
                    d_err_z = 0.0
                self._prev_err_z = err_z
                self._last_pid_time = now
                v_z_pid = self.kp_z * err_z + self.kd_z * d_err_z

                # cmd_vel 时效保护: move_base 停发则悬停
                if (now - self.cmd_vel_time).to_sec() > self.cmd_vel_timeout:
                    v_x, v_y = 0.0, 0.0
                else:
                    v_x = self.cmd_vel_data.linear.x
                    v_y = self.cmd_vel_data.linear.y

                self.send_velocity_setpoint(v_x, v_y, v_z_pid, yaw_rate=0.0)

                # 到达判定: XY + 松弛的 Z
                dist = self.get_distance_to_target(x, y)
                z_err = abs(err_z)
                if dist < self.waypoint_xy_tol and z_err < 2.0 * self.waypoint_z_tol:
                    rospy.loginfo("到达航点! 距离 %.2f m, 高度误差 %.2f m", dist, z_err)
                    arrived = True
                    break
                if self._check_timeout(start_time, self.waypoint_timeout):
                    rospy.logwarn("航点超时 (%.0fs), 还差 %.2f m, 跳过",
                                  self.waypoint_timeout, dist)
                    break
                self.rate.sleep()

            # 悬停: 到达 → 停目标; 超时 → 停当前 (未到达的目标可能在障碍物里!)
            if arrived:
                hx, hy, hz = x, y, z
            else:
                hx = self.current_position.pose.position.x
                hy = self.current_position.pose.position.y
                hz = self.current_position.pose.position.z
            rospy.loginfo("悬停 %.1fs @ (%.2f, %.2f, %.2f)", hover_time, hx, hy, hz)
            if not self._hold_at(hx, hy, hz, hover_time):
                return False
            rospy.loginfo("航点完成。")
            return arrived
        finally:
            self.cancel_goal()  # 撤销 move_base 目标, 防止其继续输出

    # ===================== 降落 =====================

    def land_at_current_position(self):
        rospy.loginfo("启动安全降落...")
        if not self.pose_received:
            rospy.logerr("无位姿数据, 直接请求 AUTO.LAND。")
            self._switch_mode('AUTO.LAND')
            return
        if not self.airborne:
            rospy.loginfo("高度 %.2f m, 视为在地面。",
                          self.current_position.pose.position.z)
            self._switch_mode('AUTO.LAND')
            self._try_disarm()
            return

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        current_z = self.current_position.pose.position.z

        descent_step = 0.02   # 20Hz ≈ 0.4 m/s
        safe_land_z = 0.15

        if current_z > safe_land_z:
            rospy.loginfo("锁定 XY+偏航, 缓慢下降...")
            start_time = rospy.Time.now()
            target_z = current_z
            while not rospy.is_shutdown() and current_z > safe_land_z + 0.05:
                target_z = max(target_z - descent_step, safe_land_z)
                self.send_position_setpoint(lock_x, lock_y, target_z, yaw=lock_yaw)
                current_z = self.current_position.pose.position.z
                if not self._pose_is_fresh(max_age=1.5):
                    rospy.logwarn("降落中位姿失效, 立即交给飞控。")
                    break
                if self._check_timeout(start_time, self.land_timeout):
                    rospy.logwarn("下降超时, 强制 AUTO.LAND。")
                    break
                self.rate.sleep()

        self._switch_mode('AUTO.LAND')
        rospy.loginfo("已请求 AUTO.LAND, 等待落地...")
        self._wait_landed(30.0)
        self._try_disarm()

    def handle_emergency(self):
        """紧急情况处置 (只在主线程调用)"""
        if self._emergency_reason == 'low_battery' and self.low_battery_action == 'rtl':
            rospy.loginfo("低电量: 切换 PX4 AUTO.RTL 返航...")
            self._switch_mode('AUTO.RTL')
            self._wait_landed(60.0)
            self._try_disarm()
        else:
            self.land_at_current_position()

    def _wait_landed(self, timeout=30.0):
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_position.pose.position.z < 0.05:
                rospy.loginfo("已落地。")
                return True
            if not self.current_state.connected:
                rospy.logwarn("连接断开, 停止等待 (飞控 failsafe 接管)。")
                return False
            if self._check_timeout(start_time, timeout):
                rospy.logwarn("等待落地超时。")
                return False
            self.rate.sleep()
        return False

    def _try_disarm(self):
        try:
            if self.arm_client(False).success:
                rospy.loginfo("已上锁。")
        except rospy.ServiceException as e:
            rospy.logwarn("上锁请求失败: %s", e)

    def _switch_mode(self, mode):
        try:
            resp = self.set_mode_client(custom_mode=mode)
            if resp.mode_sent:
                rospy.loginfo("模式切换成功: %s", mode)
            else:
                rospy.logerr("模式切换失败: %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("模式切换异常 (%s): %s", mode, e)

    # ===================== move_base 目标 =====================

    def get_goal(self, x, y):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.goal_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        rospy.loginfo("发送 move_base 目标: (%.2f, %.2f)", x, y)

    def cancel_goal(self):
        try:
            self.goal_cancel_pub.publish(GoalID())  # 空 ID = 取消全部目标
        except Exception:
            pass


# ===================== 主程序 =====================

def main():
    rospy.init_node('navigation_controller', anonymous=True)
    nav = NavigationController()

    try:
        # 1. 起飞
        if not nav.takeoff():
            rospy.logerr("起飞失败, 终止任务。")
            if nav.emergency:
                nav.handle_emergency()
            elif nav.airborne:   # 起飞超时时可能已在半空, 必须降落
                nav.land_at_current_position()
            return

        # 2. 读航点
        try:
            waypoints = NavigationController.load_waypoints(nav.waypoint_file)
        except (IOError, OSError):
            rospy.logerr("打开航点文件失败: %s", nav.waypoint_file)
            nav.land_at_current_position()
            return

        if not waypoints:
            rospy.logwarn("航点为空, 悬停 3 秒后降落。")
            nav.hold_position(3.0)
            nav.land_at_current_position()
            return

        # 3. 逐航点导航
        rospy.loginfo("共 %d 个航点, 开始执行...", len(waypoints))
        for i, (x, y, z, t) in enumerate(waypoints):
            if nav.emergency or nav.shutdown_requested:
                break
            rospy.loginfo("---- 航点 %d/%d: (%.2f, %.2f, %.2f) 悬停 %.1fs ----",
                          i + 1, len(waypoints), x, y, z, t)
            if not nav.navigation_target(x, y, z, t):
                if not (nav.emergency or nav.shutdown_requested):
                    rospy.logwarn("航点 %d 未到达, 继续下一个。", i + 1)

        # 4. 收尾
        if nav.emergency:
            nav.handle_emergency()
        elif nav.shutdown_requested:
            rospy.loginfo("收到中断, 安全降落...")
            nav.land_at_current_position()
        else:
            rospy.loginfo("所有航点完成, 3 秒后降落...")
            nav.hold_position(3.0)
            if nav.emergency:
                nav.handle_emergency()
            else:
                nav.land_at_current_position()

    except Exception:
        rospy.logerr("发生未预期异常, 尝试降落:\n%s", traceback.format_exc())
        try:
            if nav.airborne:
                nav.land_at_current_position()
        except Exception:
            rospy.logerr("异常降落也失败了, 请手动接管!")
    finally:
        rospy.loginfo("导航任务结束。")


if __name__ == "__main__":
    main()
