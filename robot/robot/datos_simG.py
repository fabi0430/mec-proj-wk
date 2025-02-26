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

class datos_simG(Node):
    def __init__(self):

        super().__init__('datos_simG')
        self.publisher1 = self.create_publisher(Float64, 'J1', 10)
        self.publisher2 = self.create_publisher(Float64, 'J2', 10)
        self.publisher3 = self.create_publisher(Float64, 'J3', 10)
        self.publisher4 = self.create_publisher(Float64, 'J4', 10)
        self.publisher5 = self.create_publisher(Float64, 'J5', 10)
        self.publisher6 = self.create_publisher(Float64, 'J6', 10)
        
        self.recibir_datos_ = self.create_subscription(Twist,"Joints",self.pose_callback,10)

    def pose_callback(self,msg:Twist):

        msgJ1 = Float64()
        msgJ2 = Float64()
        msgJ3 = Float64()
        msgJ4 = Float64()
        msgJ5 = Float64()
        msgJ6 = Float64()

        J1 = msg.linear.x
        J2 = msg.linear.y
        J3 = msg.linear.z
        J4 = msg.angular.x

        if(msg.angular.y == 120.0):
            J5 = 0.0
            J6 = 0.0
        elif(msg.angular.y == 230.0):
            J5 = 0.21
            J6 = -0.21
        else:
            print("Error")

        J1 = J1 - 180.0
        J2 = J2 - 180.0
        J3 = J3 - 180.0
        J4 = J4 - 134 - 90

        J1 = np.deg2rad(J1)
        J2 = np.deg2rad(J2)
        J3 = np.deg2rad(J3)
        J4 = np.deg2rad(J4)

        print("J1: " + str(J1))

        msgJ1.data = float(J1)
        msgJ2.data = float(J2)
        msgJ3.data = float(J3)
        msgJ4.data = float(J4)
        msgJ5.data = float(J5)
        msgJ6.data = float(J6)
        
        self.publisher1.publish(msgJ1)
        self.publisher2.publish(msgJ2)
        self.publisher3.publish(msgJ3)
        self.publisher4.publish(msgJ4)
        self.publisher5.publish(msgJ5)
        self.publisher6.publish(msgJ6)

def main(args=None):
    rclpy.init(args=args)
    node = datos_simG()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
