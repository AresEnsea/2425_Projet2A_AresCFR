#!/usr/bin/env python3

import serial
import time
import rclpy
from rclpy.node import Node

class SimpleSender(Node):

    def __init__(self):
        super().__init__('simple_sender')

def main(args=None):
    rclpy.init(args=args)
    simple_sender = SimpleSender()

    # Connexion série à UART2
    uart2_port = '/dev/serial0'  # Assurez-vous que cela correspond à votre UAR>
    baudrate = 115200
    timeout = 1

    # Initialisation de la connexion série
    try:
        ser = serial.Serial(uart2_port, baudrate, timeout=timeout)
    except serial.SerialException as e:
        simple_sender.get_logger().info(f"Error opening serial port: {e}")
        exit()

    messages = ["00000000\n\r\0","00000000\n\r\0","04000000\n\r\0","08000000\n\r\0","12000000\n\r\0","12000000\n\r\0","10000400\n\r\0","08000800\n\r\0","04001200\n\r\0","00000800\n\r\0","00000400\n\r\0","00000000\n\r\0","04000400\n\r\0","08000800\n\r\0","12001200\n\r\0"]
    for message in messages:
        simple_sender.get_logger().info(f"Sending message: {message.strip()}")
        ser.write(message.encode())
        time.sleep(1)  # Attendre 5 secondes entre les messages
    ser.close()  # Fermer la connexion série
    rclpy.shutdown()

if __name__ == '__main__':
    while 1:
        main()
