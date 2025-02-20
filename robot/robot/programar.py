#! /usr/bin/env python3
import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist

class Programar(Node):

    def __init__(self):
        super().__init__("programar")
        self.enviar_datos = self.create_publisher(Twist, "Posicion", 10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)

        self.g = float(0.0)

        self.Points = []


    def send_velocity_command(self):

        print("Seleccione una opción: \n\t1.- Nuevo punto.\n\t2.- Abrir pinzas.\n\t3.- Cerrar pinzas.\n\t4.- Suprimir punto anterior.\n\t5.- Play.")
        Seleccion = input("")

        if Seleccion == "1":
            for i in range(1):
                x = input("Posición x deseada (o escribe 's' para salir): ")
                if x.lower() == "s":
                    break

                y = input("Posición y deseada (o escribe 's' para salir): ")
                if y.lower() == "s":
                    break

                z = input("Posición z deseada (o escribe 's' para salir): ")
                if z.lower() == "s":
                    break 

                x = float(x) / 100
                y = float(y) / 100
                z = float(z) / 100

                A = [x, y, z, self.g]
                self.Points.append(A)

                msg = Twist()

                msg.linear.x = x
                msg.linear.y = y
                msg.linear.z = z
                msg.angular.x = self.g

                self.enviar_datos.publish(msg)

                time.sleep(0.5)

        elif Seleccion == "2":
            self.g = float(1.0)

            try:
                A = [self.Points[-1][0],self.Points[-1][1],self.Points[-1][2], self.g]
            except:
                A = [0.0,0.0,0.0,self.g]

            self.Points.append(A)

        elif Seleccion == "3":
            self.g = float(0.0)

            try:
                A = [self.Points[-1][0],self.Points[-1][1],self.Points[-1][2], self.g]
            except:
                A = [0.0,0.0,0.0,self.g]

            self.Points.append(A)

        elif Seleccion == "4":

            self.Points = self.Points[:-1]

        elif Seleccion == "5":

            print("\nPosiciones guardadas:")
            for i, posicion in enumerate(self.Points, start=1):
                print(f"{i}: {posicion}")

                msg = Twist()

                msg.linear.x = posicion[0]
                msg.linear.y = posicion[1]
                msg.linear.z = posicion[2]
                msg.angular.x = posicion[3]

                self.enviar_datos.publish(msg)

                time.sleep(1)
        else:
            print("\n\tSELECCION INCORRECTA\n")

def main(args = None):
    rclpy.init(args=args)

    node = Programar()
    rclpy.spin(node)

    rclpy.shutdown()