# ros_gz_gazebo_manager

This is a fork of ros_ign_gazebo_manager, where we take the code and update it to be used with ROS Humble and Ignition Gazebo Fortress, as well as some other functionalities, such as world name and sdf as parameters

`ros_gz_gazebo_manager` : control Ignition Gazebo with ROS python,

* for example, you can resume or pause  Ignition Gazebo, in addition, you can also create model, delete model and set pose of model with ROS code.
*  `gz_gazebo_interface.py`  is implemented to  control Ignition Gazebo as ROS client Node, so it's convenient to control Ignition Gazebo with python (sometimes, python is more flexible to do some tasks)

> only C++ is supported in `gz-transport` , so we can't use python to communicate with Ignition Gazebo directly. 

 `ros_gz_gazebo_manager` make ROS as  bridge to implement this function indirectly.

![](arch.png)

### Usage/example

Environment

* ROS2: Humble
* Ignition Gazebo: Fortress
* ros_ign:ros2 [https://github.com/ignitionrobotics/ros_ign/tree/ros2](https://github.com/ignitionrobotics/ros_ign/tree/ros2)

launch a empty world In Ignition Gazebo and `gz_gazebo_manager` node

```bash
ros2 launch ros_gz_gazebo_manager test.launch.py 
```

* currently, the  world name is hard-code with `default` .
* the ros services `gz/default/control`,  `gz/default/create` . etc will be created.

control Ignition Gazebo with python script

```bash
ros2 run ros_gz_gazebo_manager gz_demo.py 
```