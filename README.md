# ros-workspace
## Cloning
When first cloning, use 
```git clone --recurse-submodules <url>```
If you forget to get the submodules, it will not compile. If you already cloned it, you can run 
```git submodule update --init --recursive```
after the fact.
## Installing the ros dependencies 
Setup required for ignition simulations
```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get install ignition-fortress
```
Setup required for ignition ros2_control
```export IGNITION_VERSION=fortress```

Setup required for ouster lidar
```
sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2
```
```
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
```
Setup required for sbg IMU
```
sudo adduser $USER dialout
```
This following will install the dependencies like navigation2, ros2_control, etc on to the system.<br>
```sudo rosdep init```<br>
```rosdep update```<br>
Navigate to the ros-workspace then <br>
```rosdep install --from-paths src --ignore-src -r -y```

## Building the workspace
```colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```

## Sourcing the build in terminal
```source install/setup.bash``` (Assuming you are in ros-workspace) 
</br>

To not do this each time you open up a new terminal go to the 

```nano ~/.bashrc```, and add ```source {full path to your setup.bash file}```. Ex. ```source /home/ros-workspace/install/setup.bash```

## Simulation
To launch the simulation the following commands are used

``` ros2 launch snowplow simulation.launch.py ``` - Launches the gazebo simulation and the ros nodes for the state of the robot

``` ros2 launch snowplow visualization.launch.py ``` - Brings up rviz with some preconfigured panels and visualizations

``` ros2 launch snowplow slam_localization.launch.py ``` - Starts up the SLAM toolbox and loads in the defined map

``` ros2 launch snowplow slam_navigation.launch.py ``` - Starts up the nav2 stack components like planner, waypoint follower, behaviour server, etc. At this point, you can use the rviz, 'Set nav2 goal' to move the robot autonomously to a point

</br> 

You will notice that `slam_navigation.launch.py` will bring in a pre-existing map are into view when you look at rviz. If you want to start from scratch and not load in a pre-defined area, look for
```
slam_file = os.path.join(pkg_share, 
                        "config","slam",
                        "localization_params_online_async.yaml")
```

and replace the last line to `mapper_params_online_async.yaml`. 

## Physical 
### Setup 
#### SBG Systems 
To be able to communicate with the device, be sure that your user is part of the dialout group.
Once added, restart your machine to save and apply the changes.
``` sudo adduser $USER dialout ```

### Driving the vehicle
#### Controller mode
If you want to just run the robot using the remote control, run 2 things BUT IMPORTANTLY, run the joystick.launch.py and verify that the /cmd_vel_joy value shows 0s for x,y,z. Sometimes, it locks up to 1s at which point, you have to just stop and restart the launch until it shows all 0s.

``` ros2 launch snowplow joystick.launch.py   ```

<b>Once you have verified</b>, you can launch the following

``` ros2 launch snowplow physical_controls.launch.py ```

#### Keyboard mode 

For the keyboard control, use this command

```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel_unstamped```

instead of running 

```ros2 launch snowplow joystick.launch.py```

#### Autonomous mode
There are two different parts to this. 

1. Setting up the components for autonomous navigation

2. Sending the commands of where to go

Once you have manually driven the robot to the center of the starting zone, kill all the nodes and start launching the following

#

```ros2 launch snowplow physical_simulation.launch.py``` - Launches the physical component drivers (IMU+GPS, Lidar, motor controller) and ros nodes for state of the robot

```ros2 launch snowplow visualization.launch.py``` - Launches rviz. Shows what the robot sees and where it thinks it is

```ros2 launch snowplow physical_localization_.launch.py``` - Starts up the SLAM toolbox and loads in the competition zone map

```ros2 launch snowplow physical_navigation.launch.py``` - Starts up the nav2 stack components. 

#
<b>Once you have confirmed the location to match closely with where the rviz is showing, run </b>

```ros2 launch snowplow physical_plowpath.launch.py``` - This sends all the strategically placed waypoints for the robot to autonomously navigate to.

<b>Important note</b>: When the estop is pressed, the ros2 nodes are not aware of this and will continue to send signals to move. This means that the robot will continue moving once the e-stop is released so make sure if you don't want the robot to continue, kill the physical_navigation.launch.py process by ctrl+c for that terminal.