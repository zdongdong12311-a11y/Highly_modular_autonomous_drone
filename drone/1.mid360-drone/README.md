linux版本：20.04
ros版本：noetic
记载电脑：orangepi5max
激光雷达：mid-360

一、运行mid360
1、将mid360的一分三航空线的网线插口插入机载电脑的网口中，然后给mid360上电。

2、配网：
（1）查看可用网卡：
ls /sys/class/net/
一般为：enp0s3、eth0、enP3p49s0三种
（2）配网：
sudo ip addr add 192.168.1.50/24 dev enP3p49s0
sudo ip link set enP3p49s0 up
（3）测试：
ping 192.168.1.1xx (xx为mid360后面的SN码的后两位)
或者使用Ubuntu自带的图形化界面配置

3、安装并运行Livox-SDK2：
（1）安装编译
mkdir livox_ws
cd livox_ws
mkdir src
安装cmake：sudo apt install cmake（安装过忽略）
mkdir 3rd_party
cd 3rd_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
如果要删除：
sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
sudo rm -rf /usr/local/include/livox_lidar_*

（2）修改json文件：
进入livox_ws/3rd_party/Livox-SDK2/samples /livox_lidar_quick_start这个文件夹，找到mid360_config.json，把 host_ip 改成 192.168.1.50

（3）运行Livox-SDK2示例：
进入livox_ws/3rd_party/Livox-SDK2/build/samples/livox_lidar_quick_start这个文件夹运行如下代码:
./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/mid360_config.json
运行成功会有数据流一直发（如果不是的话可能IP错了）

4、安装并运行livox_ros_driver2：
承接上一个工作空间
（1）安装编译
cd src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git 
./build.sh ROS1
（2）更改json文件，进入config文件夹，找到MID360_config.json文件，把里面的host_net_info的四个IP地址改成192.168.1.50，然后lidar_configs里面的IP地址改成192.168.1.1xx，xx为你的mid360序列号的最后两位（序列号在雷达侧面的二维码下）

（3）运行：
source ../../devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch livox_ros_driver2 rviz_MID360.launch
运行成功之后有点云出现

二、运行fast-lio2
1、安装编译：
mkdir fast_lio2_ws
cd fast_lio2_ws
mkdir src
cd src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
来到这个文件夹fast_lio2_ws/src/FAST_LIO，找到里面的CMakeLists.txt，把里面的livox_ros_driver改成livox_ros_driver2
进入src文件夹里面找到laserMapping.cpp，把里面所有的的livox_ros_driver改成livox_ros_driver2
把preprocess.h和preprocess.cpp文件里面的所有的的livox_ros_driver改成livox_ros_driver2
catkin_make
如果提示找不到livox_ros_driver2
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/你的livox_ws工作空间/devel（最好在一个终端操作或写入bashrc）
source devel/setup.bash
如果你没安装eigen库和PCL库，你就得跟着源工程的readme安装
2、运行：
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360.launch

三、新建工作空间将lidar_to_mavros文件夹复制到src下
mkdir trans_ws/src
cp ~/lidar_to_mavros ~/tran_ws/src
cd trans_ws
catkin_make
source devel/setup.bash

四、编辑.bashrc:
# 编辑 bashrc
nano ~/.bashrc
# 在最下面添加（路径根据你实际情况修改）
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash --extend
source ~/fast_lio_ws/devel/setup.bash --extend
source ~/my_ws/devel/setup.bash --extend
# 刷新变量
source ~/.bashrc

五、调飞控参数：
EKF2_EV_CTRL 开启水平、垂直位置和偏航融合。
EKF2_HGT_MODE 改成 Vision。
EKF2_GPS_CTRL 全部关闭。
需要注意的是 EKF2_EV_DELAY 不能直接填 ，Mid-360 扫描周期 100ms 加上 FAST-LIO2 处理时间，实际延迟很可能在 80~150ms 之间，建议用 `rostopic delay` 实测后再填。
EKF2_EV_POS_X/Y/Z 要填你的实际安装外参，EKF2_EVP_NOISE 和 EKF2_EVA_NOISE 也要配合调。

六、初步测试：
terminal1：
roslaunch lidar_to_mavros lidar_to_mavros.launch
观看odom和px4位姿数据是否准确，漂移。
拿着飞机绕一圈，观看位姿是否与初始位姿出现偏差。
注意：为后续避障做准备这里需要把mapping_mid360.launch里面的rviz节点去掉。

