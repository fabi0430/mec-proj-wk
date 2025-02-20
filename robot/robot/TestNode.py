#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class TestNode(Node):
    def __init__(self):
        super().__init__("TestNode")
        self.recibir_datos_ = self.create_subscription(Int32,"/keyboard/keypress",self.pose_callback,10)

    def pose_callback(self, msg:Int32):
        print(msg)



def main(args = None):
    rclpy.init(args=args)

    node = TestNode()
    rclpy.spin(node)

    rclpy.shutdown()