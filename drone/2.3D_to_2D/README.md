mkdir 3D_to_2D_ws/src
cd 3D_to_2D_ws/src
git clone https://github.com/ros-perception/pointcloud_to_laserscan.git -b lunar-devel
catkin_make 
source ~/.bashrc #假设你已经把工作空间添加到bashrc文件
cp ./point_to_scan.launch 3D_to_2D_ws/pointcloud_to_laserscan-lunar-devel/pointcloud_to_laserscan-lunar-devel/launch
roslaunch pointcloud_to_laserscan point_to_scan.launch