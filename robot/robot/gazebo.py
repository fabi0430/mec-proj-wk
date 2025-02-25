#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time

from launch.actions import ExecuteProcess
from std_msgs.msg import Float64

import subprocess

class gazebo(Node):
    def __init__(self):
        super().__init__('gazebo')
        self.publisher = self.create_publisher(Float64, 'J1', 10)
        self.recibir_datos_ = self.create_subscription(Twist,"Joints",self.pose_callback,10)

        subprocess.Popen(['gnome-terminal', '--', 'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/J1@std_msgs/msg/Float64@gz.msgs.Double'])

    def pose_callback(self,msg:Twist):

        msgJ1 = Float64()

        J1 = msg.linear.x


        J1 = J1 - 180.0

        J1 = np.deg2rad(J1)

        print("J1: " + str(J1))

        msgJ1.data = float(J1)
        
        self.publisher.publish(msgJ1)

def main(args=None):
    rclpy.init(args=args)
    node = gazebo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
