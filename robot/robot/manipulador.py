#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener
import numpy as np
import time

class Manipulador(Node):
    def __init__(self):
        super().__init__("manipulador")
        self.enviar_datos = self.create_publisher(Twist, "Posicion", 10)
        self.recibir_datos_ = self.create_subscription(Twist, "Posicion", self.pose_callback, 10)
        self.current_pos = np.zeros(4)

        self.current_pos[0] = 0.15
        self.current_pos[1] = 0
        self.current_pos[2] = 0.08
        self.current_pos[3] = 0.0

        self.speed = 0.005
        self.last_pressed_time = 0  # Control de tiempo de la última tecla presionada

    def pose_callback(self, msgPos: Twist):
        self.current_pos[0] = msgPos.linear.x
        self.current_pos[1] = msgPos.linear.y
        self.current_pos[2] = msgPos.linear.z
        self.current_pos[3] = msgPos.angular.x

        #self.get_logger().info(f"Posición actual: {self.current_pos}")

    def start_keyboard_listener(self):
        # Inicia el listener en un hilo separado
        listener = Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    def on_press(self, key):
        current_time = time.time()
        
        # Solo procesamos la tecla si ha pasado suficiente tiempo desde la última tecla
        if current_time - self.last_pressed_time >= 0.12:  # Por ejemplo, 0.1 segundos de intervalo
            try:
                if key.char == 'w':  # Mover hacia adelante
                    self.current_pos[0] += self.speed
                    self.publicar_posicion()
                elif key.char == 's':  # Mover hacia atrás
                    self.current_pos[0] -= self.speed
                    self.publicar_posicion()
                elif key.char == 'a':  # Mover hacia la izquierda
                    self.current_pos[1] += self.speed
                    self.publicar_posicion()
                elif key.char == 'd':  # Mover hacia la derecha
                    self.current_pos[1] -= self.speed
                    self.publicar_posicion()
                elif key.char == 'e':  # Mover hacia arriba
                    self.current_pos[2] += self.speed/2
                    self.publicar_posicion()
                elif key.char == 'q':  # Mover hacia abajo
                    self.current_pos[2] -= self.speed/2
                    self.publicar_posicion()
                elif key.char == 'f':
                    if(self.current_pos[3] == 1):
                        self.current_pos[3] = 0.0
                    else:
                        self.current_pos[3] = 1.0

                    self.publicar_posicion()
            except AttributeError:
                pass  # Manejar teclas especiales que no tienen el atributo `char`

            self.last_pressed_time = current_time  # Actualiza el tiempo de la última tecla presionada

    def on_release(self, key):
        if key == Key.esc:  # Detener el listener con `Esc`
            return False

    def publicar_posicion(self):
        msg = Twist()
        msg.linear.x = self.current_pos[0]
        msg.linear.y = self.current_pos[1]
        msg.linear.z = self.current_pos[2]
        msg.angular.x = self.current_pos[3]

        self.enviar_datos.publish(msg)

        if(msg.angular.x == 1.0):
            print(f"Posición actual: [{msg.linear.x:.3f} / {msg.linear.y:.3f} / {msg.linear.z:.3f} / abierto]")
        else:
            print(f"Posición actual: [{msg.linear.x:.3f} / {msg.linear.y:.3f} / {msg.linear.z:.3f} / cerrado]")

def main(args=None):
    rclpy.init(args=args)
    node = Manipulador()

    # Inicia el listener de teclado
    node.start_keyboard_listener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
