#!/bin/python3
import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose


from ros_gz_gazebo_manager.gz_gazebo_interface import  GzGazeboInterface


if __name__ == "__main__":
    rclpy.init()
    gz = GzGazeboInterface()
    # Spin MoveIt2 node in the background
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(gz)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    input_msg="#1:resume, 2:pause, 3:create model, 4:move model 5:delete model >> "
    while(rclpy.ok()):
        mode = input(input_msg)
        mode = int(mode)
        if mode == 1:
            gz.resume()
        elif mode == 2:
            gz.pause()
        elif mode ==3:
            pose=Pose()
            pose.position.z=1.1
            gz.create_model("obj1",pose,"https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/LitterBin",is_wait=True)
        elif mode == 4:
            pose=Pose()
            pose.position.z=1.905
            gz.set_model_pose("obj1",pose)
        elif mode == 5:
            gz.remove_model("obj1")
    rclpy.shutdown()     