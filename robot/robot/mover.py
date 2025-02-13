#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class Mover(Node):

    def __init__(self):
        super().__init__("mover")
        self.enviar_datos = self.create_publisher(Twist, "Posicion", 10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)

    def send_velocity_command(self):

        user_input_x = input("Posición x deseada: ")

        user_input_y = input("Posición y deseada: ")

        user_input_z = input("Posición z deseada: ")

        pos_deseada_x = float(user_input_x) / 100
        pos_deseada_y = float(user_input_y) / 100
        pos_deseada_z = float(user_input_z) / 100

        msg = Twist()
        msg.linear.x = pos_deseada_x
        msg.linear.y = pos_deseada_y
        msg.linear.z = pos_deseada_z

        self.enviar_datos.publish(msg)

def main(args = None):
    rclpy.init(args=args)

    node = Mover()
    rclpy.spin(node)

    rclpy.shutdown()