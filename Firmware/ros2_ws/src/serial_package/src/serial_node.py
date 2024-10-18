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
    uart2_port = '/dev/serial0'  # Assurez-vous que cela correspond à votre UART2
    baudrate = 115200
    timeout = 1

    # Initialisation de la connexion série
    try:
        ser = serial.Serial(uart2_port, baudrate, timeout=timeout)
    except serial.SerialException as e:
        simple_sender.get_logger().info(f"Error opening serial port: {e}")
        exit()

    # Envoi du message "hello ares"
    message = "hello ares\n"
    ser.write(message.encode())
    simple_sender.get_logger().info(f"Sent message: {message.strip()}")

    # Pause pour s'assurer que le message est envoyé
    time.sleep(1)

    ser.close()  # Fermer la connexion série
    rclpy.shutdown()

if __name__ == '__main__':
    while 1:
        main()
