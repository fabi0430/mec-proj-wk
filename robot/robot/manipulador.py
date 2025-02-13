#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import numpy as np

class XboxController(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller detected!")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info("Xbox controller detected!")

        self.xbox_controller_state = [0.0, 0.0, 0.0, 0.0]
        self.dead_zone = 0.1

    def get_controller_input(self):
        pygame.event.pump()  # Process controller events

        # Read left stick X-axis (Left/Right movement)
        x_axis = self.joystick.get_axis(0)
        y_axis = self.joystick.get_axis(1)
        z_axis = self.joystick.get_axis(3)

        # Apply dead zone
        x_axis = 0 if abs(x_axis) < self.dead_zone else x_axis
        y_axis = 0 if abs(y_axis) < self.dead_zone else y_axis
        z_axis = 0 if abs(z_axis) < self.dead_zone else z_axis

        return {"x": x_axis, "y": y_axis, "z": z_axis, "grip": self.joystick.get_button(0)}

class Manipulador(Node):
    def __init__(self):
        super().__init__("manipulador")
        self.enviar_datos = self.create_publisher(Twist, "Posicion", 10)
        self.current_pos = np.array([0.15, 0.0, 0.08, 0.0])
        self.speed = 0.005

        # Initialize Xbox controller
        self.controller = XboxController()

        # Run control loop
        self.create_timer(0.1, self.controller_read)

    def controller_read(self):
        controller_input = self.controller.get_controller_input()

        # Mapping controller input to movement
        self.current_pos[0] += controller_input["y"] * self.speed  # Forward/Backward
        self.current_pos[1] -= controller_input["x"] * self.speed  # Left/Right
        self.current_pos[2] += controller_input["z"] * self.speed / 2  # Up/Down
        
        # Toggle gripper state
        if controller_input["grip"]:
            self.current_pos[3] = 1.0 if self.current_pos[3] == 0 else 0.0

        self.publicar_posicion()

    def publicar_posicion(self):
        msg = Twist()
        msg.linear.x = self.current_pos[0]
        msg.linear.y = self.current_pos[1]
        msg.linear.z = self.current_pos[2]
        msg.angular.x = self.current_pos[3]

        self.enviar_datos.publish(msg)

        estado_pinza = "abierto" if msg.angular.x == 1.0 else "cerrado"
        print(f"Posición actual: [{msg.linear.x:.3f} / {msg.linear.y:.3f} / {msg.linear.z:.3f} / {estado_pinza}]")

def main(args=None):
    rclpy.init(args=args)
    node = Manipulador()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
