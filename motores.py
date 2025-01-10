#! /usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

from dynamixel_sdk import *

TORQUE = 64
LED = 65
GOAL = 116
ACTUAL = 132

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0'  # Puerto

# Inicializar el puerto y el protocolo
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Habilitar torque
packetHandler.write1ByteTxRx(portHandler, 11, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 12, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 13, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 14, TORQUE, 1)
packetHandler.write1ByteTxRx(portHandler, 15, TORQUE, 1)

print("Dynamixel está listo.")

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

        J1 = int(ReSize(msg.linear.x))
        J2 = int(ReSize(msg.linear.y))
        J3 = int(ReSize(msg.linear.z))
        J4 = int(ReSize(msg.angular.x))
        J5 = int(ReSize(msg.angular.y))

        print("J1: " + str(round(msg.linear.x)) + "°   J2: " + str(round(msg.linear.y)) + "°   J3: " + str(round(msg.linear.z)) + "°   J4: " + str(round(msg.angular.x)) + "°   J5: " + str(round(msg.angular.y)) + "°")

        while(False):
            user = input("Aceptar movimiento? (y/n) ")
            if(user == 'y'):
                break

        packetHandler.write4ByteTxRx(portHandler, 11, GOAL, J1)
        packetHandler.write4ByteTxRx(portHandler, 12, GOAL, J2)
        packetHandler.write4ByteTxRx(portHandler, 13, GOAL, J3)
        packetHandler.write4ByteTxRx(portHandler, 14, GOAL, J4)
        packetHandler.write4ByteTxRx(portHandler, 15, GOAL, J5)

        while(True):
            JointActual1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 11, ACTUAL)
            JointActual2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 12, ACTUAL)
            JointActual3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 13, ACTUAL)
            JointActual4, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 14, ACTUAL)
            JointActual5, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 15, ACTUAL)

            #np.abs(JointActual1 - J1) < 10 and np.abs(JointActual2 - J2) < 10 and np.abs(JointActual3 - J3) < 10 and np.abs(JointActual4 - J4) < 10 and np.abs(JointActual5 - J5) < 10

            if(True):
                JointActual1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 11, ACTUAL)
                JointActual2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 12, ACTUAL)
                JointActual3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 13, ACTUAL)
                JointActual4, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 14, ACTUAL)
                JointActual5, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 15, ACTUAL)
                
                break

        msgAct = Twist()
        
        msgAct.linear.x = float(JointActual1)
        msgAct.linear.y = float(JointActual2)
        msgAct.linear.z = float(JointActual3)
        msgAct.angular.x = float(JointActual4)
        msgAct.angular.y = float(JointActual5)

        msgAct.linear.x = float(0)
        msgAct.linear.y = float(-np.pi/2)
        msgAct.linear.z = float(np.pi/2)
        msgAct.angular.x = float(np.pi/2)

        self.joints_actual.publish(msgAct)


def main(args=None):
    rclpy.init(args=args)

    node = Motores()
    rclpy.spin(node)

    rclpy.shutdown()
