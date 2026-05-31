/**
 * lidar_to_mavros.cpp
 *
 * 桥接 FAST-LIO2 里程计到 PX4 EKF2 vision_pose。
 *
 * 功能:
 *   - 订阅 /Odometry (FAST-LIO2 输出)
 *   - 发布 /mavros/vision_pose/pose (PX4 EKF2 视觉位置融合)
 *   - 保留 FAST-LIO2 原始时间戳 (若为空则使用当前 ROS 时间)
 *   - 每秒输出 LiDAR 里程计与 PX4 位姿对比日志
 *
 * 参数:
 *   - vision_frame_id (string, 默认 "camera_init"): vision_pose 的 frame_id
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

class VisionPoseBridge
{
public:
  VisionPoseBridge(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), pi_(3.14159265358979323846)
  {
    rate_ = new ros::Rate(30);

    // 参数
    nh_private.param<std::string>("vision_frame_id", vision_frame_id_, "camera_init");

    // 订阅 PX4 本地位姿 (用于对比日志)
    px4_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10,
        &VisionPoseBridge::px4PoseCallback, this);

    // 订阅 FAST-LIO2 里程计
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/Odometry", 2,
        &VisionPoseBridge::odomCallback, this);

    // 发布 vision_pose
    vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "mavros/vision_pose/pose", 10);

    // 状态初始化
    odom_received_ = false;
    first_pose_received_ = false;
    new_odom_available_ = false;

    ROS_INFO("lidar_to_mavros bridge initialized (frame_id=%s)", vision_frame_id_.c_str());
  }

  ~VisionPoseBridge()
  {
    delete rate_;
  }

  void run()
  {
    while (ros::ok())
    {
      if (!odom_received_)
      {
        ROS_WARN_THROTTLE(2.0, "No odometry from FAST-LIO2 yet!");
      }
      else if (new_odom_available_)
      {
        vision_pose_pub_.publish(estimated_pose_);
        new_odom_available_ = false;
      }

      // 定期输出对比日志 (1Hz)
      if (odom_received_)
      {
        ROS_INFO_THROTTLE(1.0,
            "\n--- LiDAR vs PX4 Pose ---\n"
            "          LidarPose   px4Pose\n"
            "x       %8.3f   %8.3f\n"
            "y       %8.3f   %8.3f\n"
            "z       %8.3f   %8.3f\n"
            "pitch   %8.2f    %8.2f\n"
            "roll    %8.2f    %8.2f\n"
            "yaw     %8.2f    %8.2f\n"
            "---------------------------",
            estimated_pose_.pose.position.x, px4_pose_.pose.position.x,
            estimated_pose_.pose.position.y, px4_pose_.pose.position.y,
            estimated_pose_.pose.position.z, px4_pose_.pose.position.z,
            lidar_attitude_.pitch, px4_attitude_.pitch,
            lidar_attitude_.roll, px4_attitude_.roll,
            lidar_attitude_.yaw, px4_attitude_.yaw);
      }

      ros::spinOnce();
      rate_->sleep();
    }
  }

private:
  struct Attitude { double pitch; double roll; double yaw; };

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    // 保留 FAST-LIO2 原始时间戳; 若为空则用当前 ROS 时间
    estimated_pose_.header = msg->header;
    if (estimated_pose_.header.stamp.isZero())
    {
      estimated_pose_.header.stamp = ros::Time::now();
    }
    estimated_pose_.header.frame_id = vision_frame_id_;
    estimated_pose_.pose = msg->pose.pose;

    // 解算欧拉角
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    lidar_attitude_.pitch = pitch * 180.0 / pi_;
    lidar_attitude_.roll  = roll  * 180.0 / pi_;
    lidar_attitude_.yaw   = yaw   * 180.0 / pi_;

    odom_received_ = true;
    first_pose_received_ = true;
    new_odom_available_ = true;
  }

  void px4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    px4_pose_.pose = msg->pose;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    px4_attitude_.pitch = pitch * 180.0 / pi_;
    px4_attitude_.roll  = roll  * 180.0 / pi_;
    px4_attitude_.yaw   = yaw   * 180.0 / pi_;
  }

  // ROS 句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Rate *rate_;
  const double pi_;

  // 话题
  ros::Subscriber px4_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  vision_pose_pub_;

  // 参数
  std::string vision_frame_id_;

  // 状态
  bool odom_received_;
  bool first_pose_received_;
  bool new_odom_available_;
  geometry_msgs::PoseStamped estimated_pose_;
  geometry_msgs::PoseStamped px4_pose_;
  Attitude lidar_attitude_ = {0, 0, 0};
  Attitude px4_attitude_   = {0, 0, 0};
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_to_mavros");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  VisionPoseBridge bridge(nh, nh_private);
  bridge.run();

  return 0;
}
