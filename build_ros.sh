echo "Building ROS nodes"

apt-get update
apt-get install ros-$ROS_DISTRO-image-transport-plugins
apt-get install ros-$ROS_DISTRO-tf2-geometry-msgs
cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
