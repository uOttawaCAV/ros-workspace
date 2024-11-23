# ros-workspace

## Installing the ros dependencies 
This will install the dependencies like navigation2, ros2_control, etc on to the system.<br>
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

## Mapping/Localization
To launch the simulation the following command is used

``` ros2 launch snowplow snowplow_bringup.launch.py ```
<br> 
To change it to localization, go into the ```slam_localization.launch.py``` and change the config location
```
slam_file = os.path.join(pkg_share, 
                        "config","slam",
                        "mapper_params_online_async.yaml")
```
from ```mapper_params_online_async.yaml``` to ```localization_params_online_async.yaml``` and vice versa 

## Physical Testing
If you want to just run the robot using the remote control, run

``` ros2 launch snowplow remote_control.launch.py ```

If you want to run all the autonomous functionality and sensor drivers, run

``` ros2 launch snowplow physical_snowplow_bringup.launch.py ``` 

## Driving the vehicle
You can drive using both the keyboard and remote control as explained above. For the keyboard control, use this command
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel_unstamped```
