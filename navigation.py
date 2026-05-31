#!/usr/bin/env python3
"""
navigation.py - 无人机自主导航控制器

基于 Livox Mid-360 + FAST-LIO2 + Cartographer 2D SLAM + move_base
实现多航点自主导航，支持：
- 自动起飞 / 航点导航 / 安全降落
- 高度 PID 保持
- 电池电压监测与低电量返航 (RTH)
- 连接丢失 / 位姿超时保护
- 航点文件支持注释行与环境变量配置
- ROS 参数服务器动态配置

用法:
    python3 navigation.py
    rosrun navigation_controller navigation.py  # 也可通过 rosrun
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


class NavigationController:
    """无人机自主导航控制器 - 支持航点导航、安全降落、电池监测"""

    def __init__(self):
        # ---- ROS 参数 (可通过 launch 文件或 rosparam 覆盖) ----
        self.target_z = rospy.get_param('~target_z', 0.0)
        self.kp_z = rospy.get_param('~kp_z', 1.5)
        self.kd_z = rospy.get_param('~kd_z', 0.0)          # 新增: D 项抑制高度振荡
        self.max_xy_speed = rospy.get_param('~max_xy_speed', 1.5)
        self.max_z_speed = rospy.get_param('~max_z_speed', 0.8)
        self.waypoint_xy_tol = rospy.get_param('~waypoint_xy_tol', 0.3)
        self.waypoint_z_tol = rospy.get_param('~waypoint_z_tol', 0.15)
        self.waypoint_timeout = rospy.get_param('~waypoint_timeout', 120.0)
        self.takeoff_timeout = rospy.get_param('~takeoff_timeout', 30.0)
        self.land_timeout = rospy.get_param('~land_timeout', 60.0)  # 新增: 降落超时
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 20.0)  # 新增: 低电量阈值 (%)
        self.takeoff_height = rospy.get_param('~takeoff_height', 0.8)  # 新增: 默认起飞高度

        # ---- 内部状态 ----
        self.current_state = State()
        self.current_position = PoseStamped()
        self.battery = BatteryStatus()
        self.pose_received = False
        self.last_pose_time = rospy.Time(0)
        self.now_yaw = 0.0
        self.cmd_vel_data = Twist()
        self.rate = rospy.Rate(20)  # 20 Hz
        self._prev_err_z = 0.0     # 上一次高度误差 (PD 控制)
        self._emergency_land_triggered = False  # 紧急降落标记
        self._home_position = None  # 起飞位置 (用于 RTH)

        # ---- 发布者 ----
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # ---- 订阅者 ----
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.current_position_callback)
        rospy.Subscriber('/mavros/battery', BatteryStatus, self.battery_callback)  # 新增

        # ---- 服务客户端 ----
        rospy.wait_for_service('/mavros/set_mode', timeout=30)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        try:
            rospy.wait_for_service('/mavros/cmd/land', timeout=5)
            self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)  # 新增
        except rospy.ROSException:
            rospy.logwarn("LAND 服务不可用，将使用模式切换降落")
            self.land_client = None

        # ---- 航点文件路径 ----
        self.waypoint_file = self._get_waypoint_path()

        # ---- 信号处理: Ctrl+C 优雅降落 ----
        signal.signal(signal.SIGINT, self._sigint_handler)

    # ===================== 回调函数 =====================

    def state_callback(self, msg):
        self.current_state = msg
        # 检测连接断开
        if not msg.connected and self.pose_received:
            rospy.logerr("MAVROS 连接断开! 尝试紧急降落...")
            self._trigger_emergency_land()

    def battery_callback(self, msg):
        """电池状态回调 - 低电量保护"""
        self.battery = msg
        if msg.percentage >= 0 and msg.percentage < self.low_battery_threshold and self.pose_received:
            rospy.logerr("低电量警告! 剩余: %.1f%%, 阈值: %.1f%%", msg.percentage, self.low_battery_threshold)
            self._trigger_emergency_land()

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data.linear.x = self._clamp(msg.linear.x, -self.max_xy_speed, self.max_xy_speed)
        self.cmd_vel_data.linear.y = self._clamp(msg.linear.y, -self.max_xy_speed, self.max_xy_speed)
        self.cmd_vel_data.angular.z = msg.angular.z
        if abs(msg.linear.x) > self.max_xy_speed or abs(msg.linear.y) > self.max_xy_speed:
            rospy.logwarn_throttle(1.0, "cmd_vel 限幅: 原始 vx=%.2f vy=%.2f", msg.linear.x, msg.linear.y)

    def current_position_callback(self, msg):
        self.current_position = msg
        self.pose_received = True
        self.last_pose_time = rospy.Time.now()
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        self.now_yaw = transformations.euler_from_quaternion(q)[2]

    # ===================== 工具函数 =====================

    def _get_waypoint_path(self):
        """查找航点文件，支持环境变量和 ROS 参数覆盖"""
        # ROS 参数最高优先级
        ros_param_path = rospy.get_param('~waypoint_file', '')
        if ros_param_path and os.path.isfile(ros_param_path):
            return ros_param_path

        # 环境变量
        env_path = os.environ.get('DRONE_WAYPOINT_FILE', '')
        if env_path and os.path.isfile(env_path):
            return env_path

        # 按优先级搜索
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(script_dir, 'point.txt'),
            os.path.expanduser('~/point.txt'),
        ]
        for path in candidates:
            if os.path.isfile(path):
                return path

        fallback = os.path.join(script_dir, 'point.txt')
        rospy.logwarn("未找到航点文件，使用默认路径: %s", fallback)
        return fallback

    def _clamp(self, value, lower, upper):
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

    @staticmethod
    def load_waypoints(filepath):
        """解析航点文件，支持 # 注释行和空行"""
        waypoints = []
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                stripped = line.strip()
                if not stripped or stripped.startswith('#'):
                    continue
                parts = stripped.split()
                if len(parts) >= 4:
                    try:
                        x, y, z, t = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
                        waypoints.append((x, y, z, t))
                    except ValueError:
                        rospy.logwarn("航点文件第 %d 行解析失败: %s", line_num, stripped)
                else:
                    rospy.logwarn("航点文件第 %d 行格式无效 (需要 x y z hover_time): %s", line_num, stripped)
        return waypoints

    # ===================== 安全保护 =====================

    def _trigger_emergency_land(self):
        """触发紧急降落 (只触发一次)"""
        if self._emergency_land_triggered:
            return
        self._emergency_land_triggered = True
        rospy.logerr("=" * 40)
        rospy.logerr("!!! 紧急降落已触发 !!!")
        rospy.logerr("=" * 40)
        self.land_at_current_position()

    def _check_emergency(self):
        """周期性检查紧急条件"""
        if self._emergency_land_triggered:
            return True
        # 位姿数据超时 (>3秒无更新)
        if self.pose_received and not self._pose_is_fresh(max_age=3.0):
            rospy.logerr("位姿数据超时 (>3s 无更新)，触发紧急降落")
            self._trigger_emergency_land()
            return True
        # MAVROS 断连
        if not self.current_state.connected and self.pose_received:
            rospy.logerr("MAVROS 连接断开，触发紧急降落")
            self._trigger_emergency_land()
            return True
        return False

    def _sigint_handler(self, signum, frame):
        """Ctrl+C 信号处理 - 尝试安全降落而非直接退出"""
        rospy.loginfo("收到中断信号，尝试安全降落...")
        if self.pose_received and self.current_position.pose.position.z > 0.15:
            self.land_at_current_position()
        sys.exit(0)

    # ===================== 飞行控制 =====================

    def send_velocity_setpoint(self, v_x_body, v_y_body, v_z, yaw_rate):
        """发送速度设定点 (body 坐标系转 local 坐标系)"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = 1479  # 忽略位置和加速度，使用速度和偏航角速率

        cos_yaw = math.cos(self.now_yaw)
        sin_yaw = math.sin(self.now_yaw)
        setpoint.velocity.x = v_x_body * cos_yaw - v_y_body * sin_yaw
        setpoint.velocity.y = v_x_body * sin_yaw + v_y_body * cos_yaw
        setpoint.velocity.z = self._clamp(v_z, -self.max_z_speed, self.max_z_speed)
        setpoint.yaw_rate = yaw_rate
        self.setpoint_pub.publish(setpoint)

    def send_position_setpoint(self, x, y, z, yaw=None):
        """发送位置设定点"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = 2552  # 忽略速度和加速度，使用位置和偏航角
        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.yaw = yaw if yaw is not None else self.now_yaw
        self.setpoint_pub.publish(setpoint)

    # ===================== 模式与解锁 =====================

    def wait_for_fcu_ready(self, timeout=30.0):
        """等待飞控连接和位姿数据就绪"""
        rospy.loginfo("等待 MAVROS 连接和本地位姿...")
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.connected and self._pose_is_fresh():
                rospy.loginfo("MAVROS 已连接，本地位姿可用。")
                return True
            if self._check_timeout(start_time, timeout):
                rospy.logerr("等待 MAVROS/本地位姿超时 (%.0fs)。", timeout)
                return False
            rospy.loginfo_throttle(2.0,
                "等待中... connected=%s pose_received=%s",
                self.current_state.connected, self.pose_received)
            self.rate.sleep()
        return False

    def set_offboard_and_arm(self):
        """切换到 OFFBOARD 模式并解锁 - 锁死当前水平位置和偏航角"""
        if not self.wait_for_fcu_ready():
            return False

        # 锁死起飞前的水平位置和偏航角
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        rospy.loginfo("预发布设定点 (5s)...")
        for _ in range(100):  # 100 * 0.05s = 5s
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)
            self.rate.sleep()

        # 切换 OFFBOARD 模式
        rospy.loginfo("设置 OFFBOARD 模式...")
        try:
            resp = self.set_mode_client(custom_mode='OFFBOARD')
            if not resp.mode_sent:
                rospy.logerr("设置 OFFBOARD 模式失败!")
                return False
            rospy.loginfo("OFFBOARD 模式已启用。")
        except rospy.ServiceException as e:
            rospy.logerr("设置 OFFBOARD 失败: %s", e)
            return False

        # 解锁
        rospy.loginfo("解锁飞行器...")
        try:
            resp = self.arm_client(True)
            if not resp.success:
                rospy.logerr("解锁飞行器失败!")
                return False
            rospy.loginfo("飞行器已解锁!")
        except rospy.ServiceException as e:
            rospy.logerr("解锁失败: %s", e)
            return False

        return True

    # ===================== 飞行任务 =====================

    def takeoff(self, height=None):
        """起飞到指定高度"""
        if height is not None:
            self.target_z = height
        else:
            self.target_z = self.takeoff_height

        rospy.loginfo("起飞至 %.2f 米...", self.target_z)

        if not self.set_offboard_and_arm():
            rospy.logerr("起飞中止: 模式/解锁失败。")
            return False

        # 记录起飞位置 (RTH 用)
        self._home_position = (
            self.current_position.pose.position.x,
            self.current_position.pose.position.y,
        )

        # 锁死水平位置和偏航角
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.send_position_setpoint(lock_x, lock_y, self.target_z, yaw=lock_yaw)

            if self._check_emergency():
                return False

            current_z = self.current_position.pose.position.z
            if abs(current_z - self.target_z) < self.waypoint_z_tol:
                rospy.loginfo("已到达目标高度: %.2f 米", current_z)
                return True

            if self._check_timeout(start_time, self.takeoff_timeout):
                rospy.logerr("起飞超时 (%.0fs)。当前高度: %.2f", self.takeoff_timeout, current_z)
                return False

            rospy.loginfo_throttle(2.0, "上升中... 当前高度: %.2f / 目标: %.2f", current_z, self.target_z)
            self.rate.sleep()

        return False

    def navigation_target(self, x, y, z, hover_time=2.0):
        """导航到指定航点 (XY 由 move_base 规划，Z 由 PD 控制)"""
        self.target_z = z

        # 等待发布者连接
        rospy.sleep(1.0)
        self.get_goal(x, y)
        rospy.loginfo("导航至 x=%.2f, y=%.2f, z=%.2f (悬停 %.1fs)", x, y, z, hover_time)

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._check_emergency():
                return False

            # 高度 PD 控制
            err_z = self.target_z - self.current_position.pose.position.z
            d_err_z = (err_z - self._prev_err_z) * 20.0  # 20Hz 下的微分
            v_z_pid = self.kp_z * err_z + self.kd_z * d_err_z
            self._prev_err_z = err_z

            # 执行导航 (yaw_rate=0 锁死偏航角)
            self.send_velocity_setpoint(
                v_x_body=self.cmd_vel_data.linear.x,
                v_y_body=self.cmd_vel_data.linear.y,
                v_z=v_z_pid,
                yaw_rate=0.0,
            )

            # 到达判定
            dist = self.get_distance_to_target(x, y)
            if dist < self.waypoint_xy_tol:
                rospy.loginfo("到达航点! 距离: %.2f m", dist)
                break

            if self._check_timeout(start_time, self.waypoint_timeout):
                rospy.logwarn("航点导航超时 (%.0fs)，跳转下一航点。", self.waypoint_timeout)
                break

            self.rate.sleep()

        # 悬停稳定
        rospy.loginfo("悬停 %.1f 秒...", hover_time)
        hover_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < hover_time:
            self.send_position_setpoint(x, y, z)
            self.rate.sleep()

        rospy.loginfo("航点任务完成。")

    def return_to_home(self):
        """返航至起飞位置"""
        if self._home_position is None:
            rospy.logwarn("无起飞位置记录，无法返航。原地降落。")
            self.land_at_current_position()
            return

        home_x, home_y = self._home_position
        rospy.loginfo("返航至起飞点 x=%.2f, y=%.2f ...", home_x, home_y)
        self.navigation_target(home_x, home_y, self.target_z, hover_time=2.0)
        self.land_at_current_position()

    def land_at_current_position(self):
        """安全降落: 先缓慢下降到安全高度，再切换 AUTO.LAND"""
        rospy.loginfo("启动安全降落...")

        if not self.pose_received:
            rospy.logerr("无位姿数据，尝试直接切换 AUTO.LAND...")
            self._switch_mode('AUTO.LAND')
            return

        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        current_z = self.current_position.pose.position.z

        descent_step = 0.02   # 20Hz 下约 0.4m/s
        safe_land_z = 0.15    # 低于此高度交给飞控 LAND

        if current_z > safe_land_z:
            rospy.loginfo("缓慢垂直降落 (锁定 XY+Yaw)...")
            start_time = rospy.Time.now()
            target_z = current_z
            while not rospy.is_shutdown() and current_z > safe_land_z + 0.05:
                target_z -= descent_step
                target_z = max(target_z, safe_land_z)
                self.send_position_setpoint(lock_x, lock_y, target_z, yaw=lock_yaw)
                current_z = self.current_position.pose.position.z
                self.rate.sleep()

                if self._check_timeout(start_time, self.land_timeout):
                    rospy.logwarn("降落超时，强制切换 AUTO.LAND。")
                    break

        # 切换 AUTO.LAND 让飞控完成最后落地和上锁
        self._switch_mode('AUTO.LAND')
        rospy.loginfo("AUTO.LAND 模式已请求，等待落地...")

        # 等待落地确认 (高度 < 0.05m 或断连)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_position.pose.position.z < 0.05:
                rospy.loginfo("已落地。")
                break
            if self._check_timeout(start_time, 30.0):
                rospy.logwarn("等待落地超时。")
                break
            self.rate.sleep()

    def _switch_mode(self, mode):
        """安全切换飞控模式"""
        try:
            resp = self.set_mode_client(custom_mode=mode)
            if resp.mode_sent:
                rospy.loginfo("模式切换成功: %s", mode)
            else:
                rospy.logerr("模式切换失败: %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("模式切换异常 (%s): %s", mode, e)

    # ===================== 目标发送 =====================

    def get_goal(self, x, y):
        """发送 2D 导航目标给 move_base"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        rospy.loginfo("发送 move_base 目标: x=%.2f, y=%.2f", x, y)


# ===================== 主程序入口 =====================

def main():
    rospy.init_node('navigation_controller', anonymous=True)
    nav = NavigationController()

    rospy.sleep(2.0)  # 等待 ROS 通信建立

    # 1. 自动起飞
    if not nav.takeoff():
        rospy.logerr("起飞失败，跳过导航。")
        return

    # 2. 读取航点文件并执行
    wp_file = nav.waypoint_file
    rospy.loginfo("读取航点文件: %s", wp_file)

    try:
        waypoints = NavigationController.load_waypoints(wp_file)
    except IOError:
        rospy.logerr("打开航点文件失败: %s", wp_file)
        rospy.loginfo("请确保航点文件存在，格式: x y z hover_time")
        nav.land_at_current_position()
        return

    if not waypoints:
        rospy.logwarn("航点文件为空，悬停 3 秒后降落。")
        rospy.sleep(3.0)
        nav.land_at_current_position()
        return

    rospy.loginfo("共 %d 个航点，开始执行...", len(waypoints))

    for i, (x, y, z, t) in enumerate(waypoints):
        if nav._emergency_land_triggered:
            break

        rospy.loginfo("=" * 50 + " 航点 #%d/%d " + "=" * 50, i + 1, len(waypoints))
        nav.navigation_target(x, y, z, t)

    # 3. 全部航点完成后降落
    if not nav._emergency_land_triggered:
        rospy.loginfo("所有航点执行完毕，3 秒后降落...")
        rospy.sleep(3.0)
        nav.land_at_current_position()

    rospy.loginfo("导航任务结束。")


if __name__ == "__main__":
    main()
