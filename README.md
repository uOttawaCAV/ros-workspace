# Installation

In addition to the base ROS installation, the following ROS packages are required:
```bash
sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2
```

Additionally, install these packages 

```bash
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
```
After installing the required dependencies, navigate to the root of the workspace and run 

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Don't forget to source the workspace after you are complete
```bash
source ros-workspace/install/setup.bash
```

For additional information regarding each package, navigate to the readme within each of the packages

# Notes
When adding new packages, please check the ros version and create a pull request that merges to the right branch (foxy, humble).
