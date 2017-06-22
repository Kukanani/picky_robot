Terminals with ROS 1 sourced

```
roscore
rosparam set robot_description "`cat $ROS2_WS/install_isolated/picky_robot/share/picky_robot/launch/ur5.urdf`" && rviz
```


Terminals with ROS 2 sourced

```
static_transform_publisher 0.15 -0.05 1.2 0 0 -2.6 world openni_depth_optical_frame
python3 /home/adam/osrf/ros2_ws/install_isolated/picky_robot/share/picky_robot/launch/ur5_launch.py
linemod_pipeline /home/adam/ros2_ws/linemod/cupnoodles_penne.yml b
picky_robot.py
depth_to_pointcloud_node
```


Terminals with BOTH sourced:

```
dynamic_bridge --bridge-all-2to1-topics
```