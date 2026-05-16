#include <ros/ros.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Odometry.h> 
#include <iostream>
#include <tf/transform_datatypes.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

using namespace std; 


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
        // 如果没有接收到里程计信息，则输出提示
        if (estimatedOdomRec_flag == false)
        {
            cout << "\033[K"
                 << "\033[31m no odom!!! \033[0m" << endl;
        }
        else
        {
            // 发布估计的位置信息
            estimatedPose.header.stamp = ros::Time::now();
            vision_pose_pub.publish(estimatedPose);

            // 输出信息到控制台
            cout << "\033[K"
                 << "\033[32m vrpn ok !\033[0m" << endl;
            cout << "\033[K"
                 << "       LidarPose                  px4Pose" << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "x      " << estimatedPose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "y      " << estimatedPose.pose.position.y << "\t\t" << px4Pose.pose.position.y << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "z      " << estimatedPose.pose.position.z << "\t\t" << px4Pose.pose.position.z << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "pitch  " << estimatedAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "roll   " << estimatedAttitude.roll << "\t\t" << px4Attitude.roll << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "yaw    " << estimatedAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
            cout << "\033[9A" << endl;
        }
        ros::spinOnce();
        rate->sleep();
    }
    cout << "\033[9B" << endl;
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
