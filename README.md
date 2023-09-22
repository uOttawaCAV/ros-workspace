# Notes
When adding new packages, please check the ros version and create a pull request that merges to the right branch (foxy, humble).

# Colon build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

#Additional dependencies
LIO-SAM
sudo apt install ros-foxy-perception-pcl \
  	   ros-foxy-pcl-msgs \
  	   ros-foxy-vision-opencv \
  	   ros-foxy-xacro
  	   
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev



