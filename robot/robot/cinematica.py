#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import roboticstoolbox as rp
from spatialmath import SE3 
from roboticstoolbox import *
import numpy as np

robot = DHRobot(
    [
        RevoluteDH(d=0.077, a=0, alpha= (-np.pi / 2), qlim=[-np.pi,np.pi]),
        RevoluteDH(d=0, a=0.13, alpha= 0, qlim=[-np.pi,0]),
        RevoluteDH(d=0, a=0.124, alpha= 0, qlim=[-3/4*np.pi,3/4*np.pi]),
        RevoluteDH(d=0, a=0.126, alpha= 0, qlim=[-1/4*np.pi,3/4*np.pi]),
    ],
    name="Robot",
)

class Cinematica(Node):

    def __init__(self):
        super().__init__("cinematica")

        self.qActual = [0, -np.pi/2, np.pi/2, np.pi/2]

        self.joints = self.create_publisher(Twist, "Joints", 10)
        self.recibir_datos_ = self.create_subscription(Twist,"Posicion",self.pose_callback,10)
        self.recibir_joints_ = self.create_subscription(Twist,"ActualJoints",self.joints_callback,10)

        print("Cinemática lista para recibir datos:")

    def send_joints(self):

        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def joints_callback(self, msgAct: Twist):

        q1 = msgAct.linear.x
        q2 = msgAct.linear.y
        q3 = msgAct.linear.z
        q4 = msgAct.angular.x

        q1 = np.deg2rad(q1)
        q2 = np.deg2rad(q2)
        q3 = np.deg2rad(q3)
        q4 = np.deg2rad(q4)

        q1 = q1 - np.pi
        q2 = q2 + np.pi/2
        q3 = q3 - np.pi/2
        q4 = q4 - np.deg2rad(134)

        if(q1 > np.pi):
            q1 = q1 - 2*np.pi
        if(q2 > np.pi):
            q2 = q2 - 2*np.pi
        if(q3 > np.pi):
            q3 = q3 - 2*np.pi
        if(q4 > np.pi):
            q4 = q4 - 2*np.pi

        T0 = np.arctan(0.024/0.128)

        q2 = q2 + T0
        q3 = q3 - T0

        #self.qActual = [q1, q2, q3, q4]

        
    
    def pose_callback(self, msg: Twist):

        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z

        if(x == 0 and y == 0 and z == 0):
            #T = SE3(0.124, 0, 0.081) * SE3.Ry(np.pi/2) * SE3.Rx(-np.pi/2)
            T = SE3(0.124, 0, 0.081) * SE3.Ry(np.pi/2)
        else:
            #T = SE3(x, y, z) * SE3.Ry(np.pi/2) * SE3.Rx(-np.pi/2)
            T = SE3(x, y, z) * SE3.Ry(np.pi/2)

        print("")
        print(msg)
        print(self.qActual)

        success = False
        contador = 0

        while(success == False):
            solver = robot.ikine_LM(T,self.qActual,30,50,mask=[1,1,1,0,1,0],joint_limits=True)
            success = solver.success

            contador = contador + 1

            if(contador == 10):
                break

        J = solver.q

        C = robot.fkine(J)

        jointsSend = Twist()

        T0 = np.arctan(0.024/0.128)

        T1 = J[0]
        T2 = J[1] - T0
        T3 = J[2] + T0
        T4 = J[3]

        T1 = T1 + np.pi
        T2 = T2 - np.pi/2
        T3 = T3 + np.pi/2
        T4 = T4 + np.deg2rad(134)


        if(T1 < 0):
            T1 = T1 + 2*np.pi
        if(T2 < 0):
            T2 = T2 + 2*np.pi
        if(T3 < 0):
            T3 = T3 + 2*np.pi
        if(T4 < 0):
            T4 = T4 + 2*np.pi

        if(T1 > 2*np.pi):
            T1 = T1 - 2*np.pi
        if(T2 > 2*np.pi):
            T2 = T2 - 2*np.pi
        if(T3 > 2*np.pi):
            T3 = T3 - 2*np.pi
        if(T4 > 2*np.pi):
            T4 = T4 - 2*np.pi

        T1 = np.rad2deg(T1)
        T2 = np.rad2deg(T2)
        T3 = np.rad2deg(T3)
        T4 = np.rad2deg(T4)

        if(x == 0 and y == 0 and z == 0):
            T1 = 180.0
            T2 = 180.0
            T3 = 180.0
            T4 = 223.0

        jointsSend.linear.x = T1
        jointsSend.linear.y = T2
        jointsSend.linear.z = T3
        jointsSend.angular.x = T4

        if(msg.angular.x == 1.0):
            #print("Abrirrrrrrrr")
            jointsSend.angular.y = 120.0
        elif(msg.angular.x == 0.0):
            #print("Cerrrrrar")
            jointsSend.angular.y = 230.0
        
        if(solver.success):
            #print("J1: " + str(round(T1)) + "°   J2: " + str(round(T2)) + "°   J3: " + str(round(T3)) + "°   J4: " + str(round(T4)) + "°")
            #print(solver)
            self.qActual = solver.q
            self.joints.publish(jointsSend)
        else:
            print("No solution found")

def main(args = None):
    rclpy.init(args=args)

    node = Cinematica()
    rclpy.spin(node)

    rclpy.shutdown()