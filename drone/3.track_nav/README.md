# cartographer
1.sudo apt-get install -y build-essential protobuf-compiler clang cmake g++ git google-mock libboost-all-dev libcairo2-dev libcurl4-openssl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev lsb-release ninja-build stow  python3-sphinx libgmock-dev libmetis-dev libceres-dev
2.sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
3.mkdir my_carto
4.cd my_carto
5.wstool init src
6.wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
7.wstool update -t src
8.sudo rosdep init
9.rosdep update
10.rosdep install --from-paths src --ignore-src --rosdistro=${noetic} -y
    (报错:ERROR: the following packages/stacks could not have their rosdep keys resolved to system dependencies: cartographer: [libabsl-dev] defined as "not available" for OS version [focal])
    (解决方案：把cartographer_ws/src/cartographer文件夹中的package.xml 文件中的第46行<depend>libabsl-dev</depend>删掉)
11.（执行决方案后”）rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
12.src/cartographer/scripts/install_abseil.sh
13.sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
   （报错：Reading package lists... Done Building dependency tree Reading state information... Done E: Unable to locate package ros-noetic-abseil-cpp）
   （正常报错，无妨）
14.（编译代码）catkin_make_isolated --install --use-ninja  -DPYTHON_EXECUTABLE=/usr/bin/python3
将lua文件和launch文件复制进cartograoher项目里面然后再编译一遍
启动：roslaunch 

# navigation
安装依赖：
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
sudo apt-get install ros-noetic-tf2-sensor-msgs
//更换密钥：
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-noetic-move-base-msgs
mkdir ros_nav_ws/src -p
cd ros_nav_ws/src
git clone https://github.com/ros-planning/navigation.git
cd ..
catkin_make
