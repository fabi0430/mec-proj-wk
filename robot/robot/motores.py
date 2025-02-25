#! /usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

from dynamixel_sdk import *

import time

GripperForce = 300 #10-800

TORQUE = 64
VELOCITY = 112
ACCELERATION = 108
PWM = 100
LED = 65
GOAL = 116
ACTUAL = 132

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0'  # Port

# Inicializar el puerto y el protocolo
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    #print("Succeeded to open the port")
    pass
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    #print("Succeeded to change the baudrate")
    pass
else:
    print("Failed to change the baudrate")
    quit()

# Habilitar torque
packetHandler.write1ByteTxRx(portHandler, 11, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 12, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 13, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 14, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 15, TORQUE, 1)

# Habilitar velocidad
packetHandler.write4ByteTxRx(portHandler, 11, VELOCITY, 25)
packetHandler.write4ByteTxRx(portHandler, 12, VELOCITY, 25)
packetHandler.write4ByteTxRx(portHandler, 13, VELOCITY, 40)
packetHandler.write4ByteTxRx(portHandler, 14, VELOCITY, 25)
packetHandler.write4ByteTxRx(portHandler, 15, VELOCITY, 30000)

# Habilitar aceleracion
packetHandler.write4ByteTxRx(portHandler, 11, ACCELERATION, 20)
packetHandler.write4ByteTxRx(portHandler, 12, ACCELERATION, 20)
packetHandler.write4ByteTxRx(portHandler, 13, ACCELERATION, 40)
packetHandler.write4ByteTxRx(portHandler, 14, ACCELERATION, 20)
packetHandler.write4ByteTxRx(portHandler, 15, ACCELERATION, 30000)

# Fuerza objetivo
packetHandler.write4ByteTxRx(portHandler, 15, PWM, GripperForce)

#print("Dynamixel está listo.")

def ReSize(x):
    return (x - 0) * (4095 - 0) / (360 - 0) + 0

def UnReSize(x):
    return (x - 0) * (360 - 0) / (4095 - 0) + 0

class Motores(Node):

    def __init__(self):
        super().__init__("motores")
        self.recibir_datos_ = self.create_subscription(Twist,"Joints",self.pose_callback,10)
        self.joints_actual = self.create_publisher(Twist, "ActualJoints", 10)

    def pose_callback(self, msg: Twist):

        inicio = time.time()

        J1 = int(ReSize(msg.linear.x))
        J2 = int(ReSize(msg.linear.y))
        J3 = int(ReSize(msg.linear.z))
        J4 = int(ReSize(msg.angular.x))
        J5 = int(ReSize(msg.angular.y))

        self.get_logger().info("J1: " + str(round(msg.linear.x)) + "°   J2: " + str(round(msg.linear.y)) + "°   J3: " + str(round(msg.linear.z)) + "°   J4: " + str(round(msg.angular.x)) + "°   J5: " + str(round(msg.angular.y)) + "°")

        packetHandler.write4ByteTxRx(portHandler, 11, GOAL, J1)
        packetHandler.write4ByteTxRx(portHandler, 12, GOAL, J2)
        packetHandler.write4ByteTxRx(portHandler, 13, GOAL, J3)
        packetHandler.write4ByteTxRx(portHandler, 14, GOAL, J4)
        packetHandler.write4ByteTxRx(portHandler, 15, GOAL, J5)

        msgAct = Twist()

        """""
        try:
            JointActual1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 11, ACTUAL)
            JointActual2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 12, ACTUAL)
            JointActual3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 13, ACTUAL)
            JointActual4, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 14, ACTUAL)
            JointActual5, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 15, ACTUAL)

            JointActual1 = np.round(UnReSize(JointActual1))
            JointActual2 = np.round(UnReSize(JointActual2))
            JointActual3 = np.round(UnReSize(JointActual3))
            JointActual4 = np.round(UnReSize(JointActual4))
        
            msgAct.linear.x = float(JointActual1)
            msgAct.linear.y = float(JointActual2)
            msgAct.linear.z = float(JointActual3)
            msgAct.angular.x = float(JointActual4)
        except:
            msgAct.linear.x = float(0)
            msgAct.linear.y = float(-np.pi/2)
            msgAct.linear.z = float(np.pi/2)
            msgAct.angular.x = float(np.pi/2)
        """""
        

        self.joints_actual.publish(msgAct)

        final = time.time()

        #print(final-inicio)


def main(args=None):
    rclpy.init(args=args)

    node = Motores()
    rclpy.spin(node)

    rclpy.shutdown()
