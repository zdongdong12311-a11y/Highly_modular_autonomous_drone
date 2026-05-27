#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from tf import transformations
import cv2
import numpy as np
import time
import serial
import threading


# ======================== 视觉识别类 (保留原有逻辑) ========================
class RobotVision:
    def __init__(self):
        # --- 1. 颜色阈值配置 (针对 640x480 优化) ---
        self.color_ranges = {
            'Red': [
                {'lower': np.array([0, 150, 70]), 'upper': np.array([10, 255, 255])},
                {'lower': np.array([170, 150, 70]), 'upper': np.array([180, 255, 255])}
            ],
            'Green': [{'lower': np.array([35, 100, 50]), 'upper': np.array([85, 255, 255])}],
            'Blue': [{'lower': np.array([100, 130, 50]), 'upper': np.array([130, 255, 255])}]
        }

    def detect_target(self, frame, target_color=None):
        """核心识别函数, 增加 target_color 参数用于第二次只识别特定颜色"""
        # 如果摄像头强制输出了高分辨率，缩放回640x480保障算法速度
        if frame.shape[1] > 640:
            frame = cv2.resize(frame, (640, 480))

        # --- 极致性能优化：放弃 CLAHE，改用高斯模糊降噪即可 ---
        # 转换为 HSV (直接转换，省去 LAB 转换的巨大开销)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detected_name = "None"
        center = None
        img_h, img_w = frame.shape[:2]

        # 如果指定了目标颜色，只遍历该颜色；否则遍历所有颜色
        search_colors = {target_color: self.color_ranges[
            target_color]} if target_color and target_color in self.color_ranges else self.color_ranges

        for color_name, masks in search_colors.items():
            full_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for m in masks:
                mask = cv2.inRange(hsv, m['lower'], m['upper'])
                full_mask = cv2.bitwise_or(full_mask, mask)

            # 减少形态学操作次数：只保留必要的闭运算来闭合圆环
            kernel = np.ones((5, 5), np.uint8)
            full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_CLOSE, kernel)

            # 寻找轮廓
            contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 1000: continue  # 640x480 下适当调小面积阈值

                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0: continue
                circularity = (4 * np.pi * area) / (perimeter ** 2)

                # 圆度判定
                if circularity > 0.5:
                    (x, y), radius = cv2.minEnclosingCircle(cnt)

                    # 检查是否完整
                    if (x - radius > 2 and x + radius < img_w - 2 and
                            y - radius > 2 and y + radius < img_h - 2):
                        center = (int(x), int(y))
                        detected_name = color_name

                        # 绘制
                        cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                        cv2.circle(frame, center, 3, (0, 0, 255), -1)
                        cv2.putText(frame, f"{color_name}", (center[0] - 20, center[1] - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        break
            if center: break

        return detected_name, center, frame


# ======================== 串口爪子控制类 (融合重构) ========================
class ClawController:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connect()

    def connect(self):
        try:
            # 1. 开启串口
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            # 2. 【核心修复】防止 DTR/RTS 信号卡死板子
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            time.sleep(1)  # 等待稳定
            # 3. 模拟一次复位脉冲，让板子从 setup() 开始跑
            self.ser.setDTR(True)
            time.sleep(0.1)
            self.ser.setDTR(False)

            rospy.loginfo(f"--- 爪子串口已连接 {self.port} ---")
            rospy.loginfo("等待 3 秒让爪子硬件初始化...")
            time.sleep(3)

            # 启动监听线程
            t = threading.Thread(target=self.read_thread, daemon=True)
            t.start()
        except Exception as e:
            rospy.logerr(f"爪子串口连接失败: {e}")

    def read_thread(self):
        """后台监听 ESP8266 的反馈"""
        while self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        rospy.loginfo(f"[爪子反馈] {line}")
            except:
                break
            time.sleep(0.01)

    def grab(self):
        """控制爪子：抓取 (发送1)"""
        rospy.loginfo("控制爪子：抓取 (发送1)")
        if self.ser and self.ser.is_open:
            self.ser.write(b'1')

    def release(self):
        """控制爪子：张开 (发送2)"""
        rospy.loginfo("控制爪子：张开 (发送2)")
        if self.ser and self.ser.is_open:
            self.ser.write(b'2')


# ======================== 导航控制器类 (修复起飞偏航) ========================
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

    def get_goal(self, x, y):
        """发送 2D 目标点给 move_base"""
        MB_goal = PoseStamped()
        MB_goal.header.stamp = rospy.Time.now()
        MB_goal.header.frame_id = "map"
        MB_goal.pose.position.x = x
        MB_goal.pose.position.y = y
        MB_goal.pose.orientation.w = 1.0
        self.goal_pub.publish(MB_goal)
        rospy.loginfo("已发送目标给 move_base: x=%.2f, y=%.2f", x, y)

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) <= 2.0 and abs(msg.linear.y) <= 2.0:
            self.cmd_vel_data = msg
        else:
            rospy.logwarn("接收到高速 cmd_vel，已限幅!")

    def current_position_callback(self, msg):
        self.current_position = msg
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.now_yaw = transformations.euler_from_quaternion(q)[2]

    def get_distance_to_target(self, target_x, target_y):
        dx = self.current_position.pose.position.x - target_x
        dy = self.current_position.pose.position.y - target_y
        return math.hypot(dx, dy)

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

    # 【修改1】：增加 yaw 参数，允许发送固定的偏航角设定点，防止追尾震荡
    def send_position_setpoint(self, x, y, z, yaw=None):
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = 2552  # 忽略速度和加速度，使用位置和偏航角

        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z

        # 如果传入了固定yaw，则使用固定值；否则使用当前实时yaw
        setpoint.yaw = yaw if yaw is not None else self.now_yaw
        self.setpoint_pub.publish(setpoint)

    def set_offboard_and_arm(self):
        """切换到OFFBOARD模式并解锁"""
        rospy.loginfo("在切换 Offboard 模式前预发布设定点...")

        # 【修改2】：在切入Offboard前，锁死当前的水平位置和偏航角
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        # 1. 必须先发布一小段时间的当前位置设定点，否则无法切入Offboard
        for i in range(100):  # 100 * 0.05s = 5秒
            self.send_position_setpoint(
                lock_x,  # 使用锁死的X
                lock_y,  # 使用锁死的Y
                self.target_z,  # 发送目标高度，准备起飞
                yaw=lock_yaw  # 使用锁死的Yaw
            )
            self.rate.sleep()

        # 2. 切换模式
        rospy.loginfo("正在设置 OFFBOARD 模式...")
        try:
            resp = self.set_mode_client(custom_mode='OFFBOARD')
            if resp.mode_sent:
                rospy.loginfo("Offboard 模式已启用!")
            else:
                rospy.logerr("设置 Offboard 模式失败!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("设置 Offboard 失败: %s" % e)
            return False

        # 3. 解锁
        rospy.loginfo("正在解锁飞行器...")
        try:
            resp = self.arm_client(True)
            if resp.success:
                rospy.loginfo("飞行器已解锁!")
            else:
                rospy.logerr("解锁飞行器失败!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("解锁失败: %s" % e)
            return False

        return True

    def takeoff(self, height):
        """起飞到指定高度"""
        self.target_z = height
        rospy.loginfo("正在起飞至 %.2f 米...", height)

        # 执行切模式和解锁
        if not self.set_offboard_and_arm():
            rospy.logerr("因模式/解锁失败导致起飞中止。")
            return False

        # 【修改3】：起飞上升过程中，继续使用锁死的X、Y和Yaw，防止起飞漂移和偏航
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw

        # 等待飞机到达目标高度
        while not rospy.is_shutdown():
            # 持续发送锁死的位置设定点保持位置和航向
            self.send_position_setpoint(
                lock_x,
                lock_y,
                self.target_z,
                yaw=lock_yaw
            )

            current_z = self.current_position.pose.position.z
            error_z = abs(current_z - self.target_z)

            if error_z < self.waypoint_z_tol:
                rospy.loginfo("已到达目标高度: %.2f 米", current_z)
                break

            rospy.loginfo("正在上升... 当前高度: %.2f", current_z)
            self.rate.sleep()

        return True

    # 增加 hover_time 参数，默认保留原来的2.0秒以防直接调用时报错
    def navigation_target(self, x, y, z, hover_time=2.0):
        self.target_z = z  # 更新目标高度

        # 等待发布者与 move_base 建立连接
        rospy.loginfo("等待发布者连接...")
        rospy.sleep(1.0)

        self.get_goal(x, y)
        rospy.loginfo("开始导航至 x=%.2f, y=%.2f, z=%.2f", x, y, z)

        while not rospy.is_shutdown():
            # 核心：高度 PID 保持
            err_z = self.target_z - self.current_position.pose.position.z
            v_z_pid = self.kp_z * err_z

            # 执行导航飞行，强制 yaw_rate=0.0 锁死偏航角
            self.send_velocity_setpoint(
                v_x_body=self.cmd_vel_data.linear.x,
                v_y_body=self.cmd_vel_data.linear.y,
                v_z=v_z_pid,
                yaw_rate=0.0
            )

            # 判断是否到达目标点
            dist = self.get_distance_to_target(x, y)
            if dist < self.waypoint_xy_tol:
                rospy.loginfo("已到达目标航点! 距离: %.2f", dist)
                break

            self.rate.sleep()

        # 到达目标点后，悬停指定时间稳定姿态 (将2.0改为hover_time)
        rospy.loginfo("在目标位置悬停 %.2f 秒...", hover_time)
        hover_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < hover_time:
            self.send_position_setpoint(x, y, z)
            self.rate.sleep()

        rospy.loginfo("航点任务完成。")

    def fly_to_point_directly(self, x, y, z, tol_xy=0.3, tol_z=0.2):
        """直接给飞控发位置点，不经过move_base避障"""
        rospy.loginfo("直接打点飞行至目标: x=%.2f, y=%.2f, z=%.2f", x, y, z)
        while not rospy.is_shutdown():
            self.send_position_setpoint(x, y, z)

            dx = self.current_position.pose.position.x - x
            dy = self.current_position.pose.position.y - y
            dz = self.current_position.pose.position.z - z
            dist_xy = math.hypot(dx, dy)
            dist_z = abs(dz)

            if dist_xy < tol_xy and dist_z < tol_z:
                rospy.loginfo("已到达直接飞行目标!")
                break
            self.rate.sleep()

    def land_at_current_position(self):
        """在当前位置切换模式自动降落"""
        rospy.loginfo("正在当前位置启动降落...")
        
        # 锁死当前XY位置和偏航角，确保缓慢垂直降落且不偏航
        lock_x = self.current_position.pose.position.x
        lock_y = self.current_position.pose.position.y
        lock_yaw = self.now_yaw
        current_z = self.current_position.pose.position.z
        
        # 设定缓慢下降的步长和最终切换AUTO.LAND的安全高度
        descent_step = 0.02  # 每次循环下降0.02米 (20Hz下约0.4m/s，缓慢下降)
        safe_land_z = 0.15   # 降到0.15米时交由飞控原生LAND逻辑落地和上锁
        
        if current_z > safe_land_z:
            rospy.loginfo("开始缓慢垂直降落，锁定偏航角...")
            while not rospy.is_shutdown() and self.current_position.pose.position.z > safe_land_z + 0.05:
                current_z -= descent_step
                if current_z < safe_land_z:
                    current_z = safe_land_z
                # 持续发送锁定XY和Yaw，Z轴缓慢减小的位置设定点
                self.send_position_setpoint(lock_x, lock_y, current_z, yaw=lock_yaw)
                self.rate.sleep()

        try:
            resp = self.set_mode_client(custom_mode='AUTO.LAND')
            if resp.mode_sent:
                rospy.loginfo("AUTO.LAND 模式已启用!")
            else:
                rospy.logerr("设置 AUTO.LAND 模式失败!")
        except rospy.ServiceException as e:
            rospy.logerr("设置 AUTO.LAND 失败: %s" % e)


# ======================== 主逻辑执行 ========================
if __name__ == "__main__":
    rospy.init_node('navigation_controller', anonymous=True)
    my_navigation = NavigationController()
    vision = RobotVision()
    claw = ClawController(port='/dev/ttyUSB0', baud=115200)

    # ================= 参数配置区 (方便调整) =================
    # 【新增】相机FOV参数 (需根据实际相机规格填写)
    CAMERA_FOV_H = 60.0  # 水平视场角 (度)
    CAMERA_FOV_V = 45.0  # 垂直视场角 (度)

    PIXEL_TO_METER_RATIO = 0.005  # 像素转物理单位的比例尺 (已不用于第7航点，保留)

    # 第二次识别任务(投放)后的位置调整参数
    DROP_OFFSET_X = 0.0  # 融合后的世界坐标系X不变
    DROP_OFFSET_Y = 0.0  # 融合后的世界坐标系Y不变
    DROP_OFFSET_Z = 0.0  # 融合后的世界坐标系Z降低0.5m

    # 【新增】11,12,13航点均未识别到目标时的备降航点序号 (对应txt中的第几行，从1开始)
    FALLBACK_LAND_WP_INDEX = 11  # 默认飞回第11个航点降落，可改为1或其他航点
    # ========================================================

    # 等待飞控连接和MAVROS启动
    rospy.sleep(2.0)

    # 1. 先自动起飞到 0.8 米
    if my_navigation.takeoff(0.7):
        # 2. 起飞成功后，读取 point.txt 执行多航点导航
        try:
            # 默认读取脚本同目录下的 point.txt
            waypoints = []
            with open('/home/orangepi/Desktop/test_points.txt', 'r') as f:
                for line in f:
                    # 跳过空行
                    if not line.strip():
                        continue
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2])
                        t = float(parts[3])
                        waypoints.append((x, y, z, t))
                    else:
                        rospy.logwarn("point.txt 中格式无效的行: %s", line.strip())

            remembered_color = None  # 用于记忆第一次识别的颜色
            early_land = False  # 标记是否提前降落

            # 遍历所有航点
            for i, (x, y, z, t) in enumerate(waypoints):
                if early_land:
                    break  # 如果已提前降落，跳出航点循环

                waypoint_idx = i + 1  # 航点序号从1开始
                rospy.loginfo("=" * 50 + f"正在执行航点 #{waypoint_idx}" + "=" * 50)

                # 执行常规2D避障导航
                my_navigation.navigation_target(x, y, z, t)

                # -------- 逻辑1：第四个航点到达后，仅执行视觉识别记颜色 --------
                if waypoint_idx == 4:
                    rospy.loginfo(">>> 已到达航点 4。启动第一次视觉任务 (识别并记录)...")
                    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FPS, 30)

                    detected_color = "None"

                    while not rospy.is_shutdown() and cap.isOpened():
                        ret, frame = cap.read()
                        if not ret: break

                        # 识别任意颜色
                        color, pos, img = vision.detect_target(frame)

                        if color != "None" and pos is not None:
                            detected_color = color
                            remembered_color = color  # 记住颜色

                            # 命令行日志输出识别到的颜色 (中文化并更醒目)
                            color_cn_map = {'Red': '红色', 'Green': '绿色', 'Blue': '蓝色'}
                            color_cn = color_cn_map.get(detected_color, detected_color)
                            rospy.loginfo("********** 第一次视觉 -> 锁定颜色: 【%s】 **********", color_cn)
                            break

                    cap.release()
                    cv2.destroyAllWindows()

                # -------- 逻辑2：第五个航点到达后，悬停3秒并控制爪子抓取 --------
                elif waypoint_idx == 5:
                    rospy.loginfo(">>> 已到达航点 5。准备抓取小球...")

                    # 在第五个航点上悬停3秒钟
                    rospy.loginfo("抓取前悬停 3 秒...")
                    hover_start = rospy.Time.now()
                    while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < 3.0:
                        my_navigation.send_position_setpoint(x, y, z)
                        my_navigation.rate.sleep()

                    # 控制爪子抓取
                    claw.grab()
                    rospy.loginfo("已抓取小球！继续带球导航...")

                # -------- 逻辑3：第11/12/13个航点到达后，限时3秒识别并决定是否降落 --------
                elif waypoint_idx in [11, 12, 13] and remembered_color is not None:
                    rospy.loginfo(">>> 已到达航点 %d。启动限时3秒视觉识别任务...", waypoint_idx)
                    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FPS, 30)

                    detected_in_this_wp = False
                    start_time = time.time()

                    while not rospy.is_shutdown() and cap.isOpened():
                        ret, frame = cap.read()
                        if not ret: break

                        # 仅识别之前记住的颜色
                        color, pos, img = vision.detect_target(frame, target_color=remembered_color)

                        if color != "None" and pos is not None:
                            detected_in_this_wp = True
                            color_cn_map = {'Red': '红色', 'Green': '绿色', 'Blue': '蓝色'}
                            color_cn = color_cn_map.get(color, color)
                            rospy.loginfo("********** 在航点 %d 识别到目标颜色: 【%s】 **********", waypoint_idx,
                                          color_cn)
                            break

                        # 超时6秒判定 (从3秒修改为6秒)
                        if time.time() - start_time > 6.0:
                            rospy.loginfo(">>> 航点 %d 识别超时(3秒)，未发现目标。", waypoint_idx)
                            break

                    cap.release()
                    cv2.destroyAllWindows()

                    # 如果识别到了，直接基于当前位置+设置的偏移量降落
                    if detected_in_this_wp:
                        # 计算投放点：当前坐标 + 配置的偏移量
                        land_x = my_navigation.current_position.pose.position.x + DROP_OFFSET_X
                        land_y = my_navigation.current_position.pose.position.y + DROP_OFFSET_Y
                        land_z = my_navigation.current_position.pose.position.z + DROP_OFFSET_Z
                        
                        rospy.loginfo("识别成功！准备飞向投放点(当前坐标+偏移量): x=%.2f, y=%.2f, z=%.2f", land_x, land_y, land_z)
                        
                        # 飞到计算出的投放点上方
                        my_navigation.fly_to_point_directly(land_x, land_y, land_z, tol_xy=0.2, tol_z=0.15)
                        
                        rospy.loginfo("到达投放点，悬停2秒后降落...")
                        hover_start = rospy.Time.now()
                        while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < 2.0:
                            my_navigation.send_position_setpoint(land_x, land_y, land_z)
                            my_navigation.rate.sleep()

                        claw.release()
                        rospy.loginfo("小球已投放！")
                        my_navigation.land_at_current_position()
                        early_land = True

                    # 如果在第13个航点仍未识别到，飞回设置的备降航点
                    elif waypoint_idx == 13:
                        rospy.logwarn("11, 12, 13航点均未识别到目标，准备飞回航点 %d 降落...", FALLBACK_LAND_WP_INDEX)
                        if 0 < FALLBACK_LAND_WP_INDEX <= len(waypoints):
                            fb_x, fb_y, fb_z, _ = waypoints[FALLBACK_LAND_WP_INDEX - 1]
                            # 直接打点飞回备降航点
                            my_navigation.fly_to_point_directly(fb_x, fb_y, fb_z)

                            rospy.loginfo("已到达降落点，悬停2秒后降落...")
                            hover_start = rospy.Time.now()
                            while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < 2.0:
                                my_navigation.send_position_setpoint(fb_x, fb_y, fb_z)
                                my_navigation.rate.sleep()
                            claw.release()
                            rospy.loginfo("小球已投放！")
                            my_navigation.land_at_current_position()
                            early_land = True
                        else:
                            rospy.logerr("设置的备降航点序号 %d 超出范围，原地紧急降落！", FALLBACK_LAND_WP_INDEX)
                            my_navigation.land_at_current_position()
                            early_land = True

            # -------- 逻辑4：所有任务完成，悬停2秒，原地降落 (如果未被提前降落打断) --------
            if not early_land:
                rospy.loginfo("所有航点和任务完成！悬停 2 秒...")
                hover_start = rospy.Time.now()
                while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < 2.0:
                    # 悬停在最后一个航点
                    my_navigation.send_position_setpoint(waypoints[-1][0], waypoints[-1][1], waypoints[-1][2])
                    my_navigation.rate.sleep()

                # 原地降落
                claw.release()
                rospy.loginfo("小球已投放！")
                my_navigation.land_at_current_position()
                rospy.loginfo("任务圆满完成!")

        except IOError:
            rospy.logerr("打开 test_pint.txt 失败！请确保文件存在。")
    else:
        rospy.logerr("起飞失败，跳过导航。")