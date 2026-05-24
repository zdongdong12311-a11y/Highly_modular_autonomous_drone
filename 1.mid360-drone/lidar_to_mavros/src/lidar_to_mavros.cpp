#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class vision_pose
{
public:
   
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);
    double pi;
  
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };

    attitude estimatedAttitude; 
    attitude px4Attitude; 

    geometry_msgs::PoseStamped px4Pose; 
    geometry_msgs::PoseStamped estimatedPose; 

    bool estimatedOdomRec_flag; 
    bool firstPoseReceived; // 用于标记是否已经接收到第一次传入的坐标

    ros::Rate *rate; 

    ros::NodeHandle nh; 
    ros::NodeHandle nh_private; 

    ros::Subscriber px4Pose_sub; 
    ros::Publisher vision_pose_pub; 
    ros::Subscriber odom_sub; 

   
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

   
    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);

   
    void start();
};


vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_) : nh(nh_), nh_private(nh_private_)
{
    
    pi = 3.1415926;
    
 
    rate = new ros::Rate(30);

    // 订阅 PX4 发送的位置信息
    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &vision_pose::px4Pose_cb, this);
    
    // 订阅里程计信息
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2, &vision_pose::estimator_odom_cb, this);
    
    // 发布估计的位置信息
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);


    estimatedOdomRec_flag = false;
    firstPoseReceived = false;


    estimatedAttitude.pitch = 0;
    estimatedAttitude.roll = 0;
    estimatedAttitude.yaw = 0;

    px4Attitude.pitch = 0;
    px4Attitude.roll = 0;
    px4Attitude.yaw = 0;
}


void vision_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  
    if (!firstPoseReceived)
    {
        estimatedPose.pose = msg->pose.pose;
        firstPoseReceived = true;
    }
    else
    {
        estimatedPose.pose = msg->pose.pose; 
    }


    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    estimatedAttitude.pitch = pitch * 180 / pi;
    estimatedAttitude.roll = roll * 180 / pi;
    estimatedAttitude.yaw = yaw * 180 / pi;

    estimatedOdomRec_flag = true;
}


void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   
    px4Pose.pose = msg->pose;


    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}


void vision_pose::start()
{
    while (ros::ok())
    {
        if (!estimatedOdomRec_flag)
        {
            ROS_WARN_THROTTLE(2.0, "No odometry from FAST-LIO2!");
        }
        else
        {
            estimatedPose.header.stamp = ros::Time::now();
            vision_pose_pub.publish(estimatedPose);

            ROS_INFO_THROTTLE(1.0,
                "\n--- Lidar vs PX4 Pose ---\n"
                "       LidarPose              px4Pose\n"
                "x      %7.3f                %7.3f\n"
                "y      %7.3f                %7.3f\n"
                "z      %7.3f                %7.3f\n"
                "pitch  %7.2f                %7.2f\n"
                "roll   %7.2f                %7.2f\n"
                "yaw    %7.2f                %7.2f\n"
                "---------------------------",
                estimatedPose.pose.position.x, px4Pose.pose.position.x,
                estimatedPose.pose.position.y, px4Pose.pose.position.y,
                estimatedPose.pose.position.z, px4Pose.pose.position.z,
                estimatedAttitude.pitch, px4Attitude.pitch,
                estimatedAttitude.roll, px4Attitude.roll,
                estimatedAttitude.yaw, px4Attitude.yaw);
        }
        ros::spinOnce();
        rate->sleep();
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_to_mavros");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");


    vision_pose vision(nh_, nh_private_);
    vision.start();

    return 0;
}
