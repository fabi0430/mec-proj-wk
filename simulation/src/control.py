#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pygame

class XboxJointController(Node):
    def __init__(self):
        super().__init__('xbox_joint_controller')

        # Initialize the publisher for /joint_states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.1, self.publish_joint_states)  # Publish every 0.1s

        # Initialize Pygame to read Xbox controller input
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller detected!")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info("Xbox controller detected!")

        # Initial joint positions (we care about joint1, joint2, joint3, joint4, and gripper)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # Starting with gripper closed (0.019)

    def publish_joint_states(self):
        pygame.event.pump()  # Update Pygame events

        # **Right Stick (X-axis)** controls joint1 (left/right)
        joint1_axis = self.joystick.get_axis(0)  # X-axis of right stick (left/right)

        # Dead zone (ignoring small stick movements)
        dead_zone = 0.1  # Threshold value (adjustable)
        if abs(joint1_axis) < dead_zone:
            joint1_axis = 0  # Ignore small movements

        # Adjust joint1 position for sensitivity
        self.joint_positions[0] += joint1_axis * 0.05  # joint1: move left/right
        # Apply limit to joint1 (±3.142)
        self.joint_positions[0] = max(-3.142, min(3.142, self.joint_positions[0]))

        # **Left Stick (Y-axis)** controls joint2 (up/down)
        joint2_axis = self.joystick.get_axis(1)  # Y-axis of left stick (up/down)

        if abs(joint2_axis) < dead_zone:
            joint2_axis = 0  # Ignore small movements

        # Adjust joint2 position for sensitivity (up/down)
        self.joint_positions[1] += joint2_axis * 0.05  # joint2: move up/down
        # Apply limit to joint2 (±1.5)
        self.joint_positions[1] = max(-1.5, min(1.5, self.joint_positions[1]))

        # **Left Trigger (LT)** controls joint3 (adjusting upwards with more pressure)
        lt_axis = self.joystick.get_axis(2)  # Left trigger (LT)

        # Adjust joint3 position based on LT pressure (positive when LT is pressed)
        self.joint_positions[2] += lt_axis * 0.05  # joint3: controlled by LT pressure

        # **Right Trigger (RT)** controls joint3 (adjusting downwards with more pressure)
        rt_axis = self.joystick.get_axis(5)  # Right trigger (RT)

        # Adjust joint3 position based on RT pressure (positive when RT is pressed)
        self.joint_positions[2] -= rt_axis * 0.05  # joint3: controlled by RT pressure
        # Apply limit to joint3 (-1.5 to 1.4)
        self.joint_positions[2] = max(-1.5, min(1.4, self.joint_positions[2]))

        # **Right Stick (Y-axis)** controls joint4 (up/down movement)
        joint4_axis = self.joystick.get_axis(3)  # Y-axis of right stick (up/down)

        if abs(joint4_axis) < dead_zone:
            joint4_axis = 0  # Ignore small movements

        # Adjust joint4 position for sensitivity (up/down movement)
        self.joint_positions[3] += joint4_axis * 0.05  # joint4: controlled by right stick Y-axis
        # Apply limit to joint4 (-1.7 to 1.970)
        self.joint_positions[3] = max(-1.7, min(1.970, self.joint_positions[3]))

        # **X Button** and **A Button** control the gripper (Open)
        x_button = self.joystick.get_button(0)  # X button
        a_button = self.joystick.get_button(1)  # A button

        # Gripper behavior:
        # If X or A is pressed, open the gripper (set to -0.010).
        if x_button or a_button:
            self.joint_positions[4] = -0.010  # Open gripper (max open)
        else:
            # Gripper closes automatically when no button is pressed
            self.joint_positions[4] = 0.019  # Close gripper (min closed)

        # Create JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint']
        msg.position = self.joint_positions

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint positions: {self.joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = XboxJointController()
    rclpy.spin(node)
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()