#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *  # Importar la librería de Dynamixel SDK

# Configuración básica

MY_DXL = 'X_SERIES'  # Modelo de Dynamixel

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0'  # Cambia según tu sistema (Linux/Mac: '/dev/ttyUSB0', Windows: 'COMX')
DXL_ID = 1

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20  # Tolerancia de movimiento

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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("Error al habilitar torque.")
    quit()
else:
    print("Torque habilitado")

print("Dynamixel está listo. Ingresa una posición (0-4095). Escribe 'exit' para salir.")

try:
    while True:
        # Pedir posición al usuario
        user_input = input("Posición objetivo: ")

        if not user_input.isdigit():  # Salir si no es un número
            break

        goal_position = int(user_input)

        # Enviar posición objetivo
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
        
        if dxl_comm_result != COMM_SUCCESS:
            print("Error de comunicación:", packetHandler.getTxRxResult(dxl_comm_result))
            continue
        if dxl_error != 0:
            print("Error del Dynamixel:", packetHandler.getRxPacketError(dxl_error))
            continue

        # Esperar hasta que alcance la posición
        while True:
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            
            if dxl_comm_result != COMM_SUCCESS:
                print("Error de comunicación:", packetHandler.getTxRxResult(dxl_comm_result))
                break
            if dxl_error != 0:
                print("Error del Dynamixel:", packetHandler.getRxPacketError(dxl_error))
                break

            print(f"[ID:{DXL_ID}] Objetivo: {goal_position}  Actual: {dxl_present_position}")
            
            if abs(goal_position - dxl_present_position) <= DXL_MOVING_STATUS_THRESHOLD:
                break

except KeyboardInterrupt:
    print("Cerrando el programa...")

finally:
    # Deshabilitar torque y cerrar puerto
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("Puerto cerrado.")