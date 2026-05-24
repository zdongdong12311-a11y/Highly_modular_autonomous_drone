#!/usr/bin/env python3
import os
import sys
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from tf import transformations


class NavigationController:
    def __init__(self):
        # 发布者
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # 订阅者
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.current_position_callback)

        # 服务客户端
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        # 成员变量
        self.current_position = PoseStamped()
        self.now_yaw = 0.0
        self.cmd_vel_data = Twist()
        self.rate = rospy.Rate(20)  # 20 Hz

        # 高度 PID 参数
        self.target_z = 0.0
        self.kp_z = 1.5

        # 到达目标点容差 (米)
        self.waypoint_xy_tol = 0.3
        self.waypoint_z_tol = 0.15  # 起飞高度容差
        self.waypoint_timeout = 120.0  # 单个航点超时 (秒)
        self.waypoint_file = self._get_waypoint_path()

    def _get_waypoint_path(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.expanduser('~/point.txt'),
            os.path.join(script_dir, 'point.txt'),
        ]
        for path in candidates:
            if os.path.isfile(path):
                return path
        return candidates[0]  # fallback 到 ~/point.txt

    def get_goal(self, x, y):
        """发送 2D 目标点给 move_base"""
        MB_goal = PoseStamped()
        MB_goal.header.stamp = rospy.Time.now()
        MB_goal.header.frame_id = "map"
        MB_goal.pose.position.x = x
        MB_goal.pose.position.y = y
        MB_goal.pose.orientation.w = 1.0
        self.goal_pub.publish(MB_goal)
        rospy.loginfo("Sent goal to move_base: x=%.2f, y=%.2f", x, y)

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) <= 2.0 and abs(msg.linear.y) <= 2.0:
            self.cmd_vel_data = msg
        else:
            rospy.logwarn("Received high speed cmd_vel, clamping!")

    def current_position_callback(self, msg):
        self.current_position = msg
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.now_yaw = transformations.euler_from_quaternion(q)[2]

    def get_distance_to_target(self, target_x, target_y):
        dx = self.current_position.pose.position.x - target_x
        dy = self.current_position.pose.position.y - target_y
        return math.hypot(dx, dy)

    def _check_nav_timeout(self, start_time):
        elapsed = (rospy.Time.now() - start_time).to_sec()
        return elapsed > self.waypoint_timeout

    def send_velocity_setpoint(self, v_x_body, v_y_body, v_z, yaw_rate):
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = 1479  # 忽略位置和加速度，使用速度和偏航角速率

        cos_yaw = math.cos(self.now_yaw)
        sin_yaw = math.sin(self.now_yaw)
        v_x_local = v_x_body * cos_yaw - v_y_body * sin_yaw
        v_y_local = v_x_body * sin_yaw + v_y_body * cos_yaw

        setpoint.velocity.x = v_x_local
        setpoint.velocity.y = v_y_local
        setpoint.velocity.z = v_z
        setpoint.yaw_rate = yaw_rate
        self.setpoint_pub.publish(setpoint)

    def send_position_setpoint(self, x, y, z):
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = 2552  # 忽略速度和加速度，使用位置和偏航角

        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.yaw = self.now_yaw
        self.setpoint_pub.publish(setpoint)

    def set_offboard_and_arm(self):
        """切换到OFFBOARD模式并解锁"""
        rospy.loginfo("Pre-publishing setpoints before Offboard switch...")
        # 1. 必须先发布一小段时间的当前位置设定点，否则无法切入Offboard
        for i in range(100):  # 100 * 0.05s = 5秒
            self.send_position_setpoint(
                self.current_position.pose.position.x,
                self.current_position.pose.position.y,
                self.target_z  # 发送目标高度，准备起飞
            )
            self.rate.sleep()

        # 2. 切换模式
        rospy.loginfo("Setting OFFBOARD mode...")
        try:
            resp = self.set_mode_client(custom_mode='OFFBOARD')
            if resp.mode_sent:
                rospy.loginfo("Offboard mode enabled!")
            else:
                rospy.logerr("Failed to set Offboard mode!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Failed to set Offboard: %s" % e)
            return False

        # 3. 解锁
        rospy.loginfo("Arming vehicle...")
        try:
            resp = self.arm_client(True)
            if resp.success:
                rospy.loginfo("Vehicle armed!")
            else:
                rospy.logerr("Failed to arm vehicle!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Failed to arm: %s" % e)
            return False

        return True

    def takeoff(self, height):
        """起飞到指定高度"""
        self.target_z = height
        rospy.loginfo("Initiating takeoff to %.2f meters...", height)

        # 执行切模式和解锁
        if not self.set_offboard_and_arm():
            rospy.logerr("Takeoff aborted due to mode/arm failure.")
            return False

        # 等待飞机到达目标高度
        while not rospy.is_shutdown():
            # 持续发送位置设定点保持高度
            self.send_position_setpoint(
                self.current_position.pose.position.x,
                self.current_position.pose.position.y,
                self.target_z
            )

            current_z = self.current_position.pose.position.z
            error_z = abs(current_z - self.target_z)

            if error_z < self.waypoint_z_tol:
                rospy.loginfo("Reached target altitude: %.2f meters", current_z)
                break

            rospy.loginfo("Ascending... Current alt: %.2f", current_z)
            self.rate.sleep()

        return True

    # 增加 hover_time 参数，默认保留原来的2.0秒以防直接调用时报错
    def navigation_target(self, x, y, z, hover_time=2.0):
        self.target_z = z  # 更新目标高度

        # 等待发布者与 move_base 建立连接
        rospy.loginfo("Waiting for publisher to connect...")
        rospy.sleep(1.0)

        self.get_goal(x, y)
        rospy.loginfo("Start navigating to x=%.2f, y=%.2f, z=%.2f", x, y, z)

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # 核心：高度 PID 保持
            err_z = self.target_z - self.current_position.pose.position.z
            v_z_pid = self.kp_z * err_z

            # 执行导航飞行
            self.send_velocity_setpoint(
                v_x_body=self.cmd_vel_data.linear.x,
                v_y_body=self.cmd_vel_data.linear.y,
                v_z=v_z_pid,
                yaw_rate=self.cmd_vel_data.angular.z
            )

            # 判断是否到达目标点
            dist = self.get_distance_to_target(x, y)
            if dist < self.waypoint_xy_tol:
                rospy.loginfo("Reached target waypoint! Distance: %.2f", dist)
                break

            if self._check_nav_timeout(start_time):
                rospy.logwarn("Waypoint navigation timed out!")
                break

            self.rate.sleep()

        # 到达目标点后，悬停指定时间稳定姿态 (将2.0改为hover_time)
        rospy.loginfo("Hovering at target position for %.2f seconds...", hover_time)
        hover_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < hover_time:
            self.send_position_setpoint(x, y, z)
            self.rate.sleep()

        rospy.loginfo("Waypoint task completed.")


if __name__ == "__main__":
    rospy.init_node('navigation_controller', anonymous=True)
    my_navigation = NavigationController()

    # 等待飞控连接和MAVROS启动
    rospy.sleep(2.0)

    # 1. 先自动起飞到 0.8 米
    if my_navigation.takeoff(0.6):
        # 2. 起飞成功后，读取 point.txt 执行多航点导航
        wp_file = my_navigation.waypoint_file
        rospy.loginfo("Loading waypoints from: %s", wp_file)
        try:
            with open(wp_file, 'r') as f:
                for line in f:
                    if not line.strip():
                        continue

                    parts = line.strip().split()
                    if len(parts) >= 4:
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2])
                        t = float(parts[3])
                        rospy.loginfo("Read waypoint: x=%.2f y=%.2f z=%.2f hover=%.2f", x, y, z, t)
                        my_navigation.navigation_target(x, y, z, t)
                    else:
                        rospy.logwarn("Invalid line format in %s: %s", wp_file, line.strip())
        except IOError:
            rospy.logerr("Failed to open waypoint file: %s", wp_file)
            rospy.loginfo("Ensure point.txt exists with format: x y z hover_time")
    else:
        rospy.logerr("Takeoff failed, skipping navigation.")
