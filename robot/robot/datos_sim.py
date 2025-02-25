#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time

class DatosSim(Node):
    def __init__(self):
        super().__init__('datos_sim')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.recibir_datos_ = self.create_subscription(Twist,"Joints",self.pose_callback,10)

        joint_positions = [0.0, 0.0, 0.0, 1.55, 0.0, 0.0]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint', 'gripper_right_joint']
        msg.position = joint_positions

        time.sleep(0.1)
        
        self.publisher.publish(msg)

    def pose_callback(self,msg:Twist):

        J1 = msg.linear.x
        J2 = msg.linear.y
        J3 = msg.linear.z
        J4 = msg.angular.x

        J1 = J1 - 180.0
        J2 = J2 - 180.0
        J3 = J3 - 180.0
        J4 = J4 - 134.0

        if(msg.angular.y == 120.0):
            J5 = 0.018
        elif(msg.angular.y == 230.0):
            J5 = 0.0
        else:
            print("Error")

        J1 = np.deg2rad(J1)
        J2 = np.deg2rad(J2)
        J3 = np.deg2rad(J3)
        J4 = np.deg2rad(J4)

        print("J1: " + str(J1) + "°   J2: " + str(J2) + "°   J3: " + str(J3) + "°   J4: " + str(J4) + "°")

        joint_positions = [J1, J2, J3, J4, J5, J5]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint', 'gripper_right_joint']
        msg.position = joint_positions
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DatosSim()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
