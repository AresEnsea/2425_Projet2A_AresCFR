#!/usr/bin/env python3

#This node receive an input from the keyboard he transforms into an UART command for the STM32

import serial
import time
import rclpy
from rclpy.node import Node
import tty
import sys
import termios

class SimpleSender(Node):
    def __init__(self):
        super().__init__('simple_sender')
        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

    def send_serial_message(self, message):
        try:
            with serial.Serial(self.uart2_port, self.baudrate, timeout=self.timeout) as ser:
                self.get_logger().info(f"Sending message: {message.strip()}")
                ser.write(message.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main(args=None):
    rclpy.init(args=args)
    simple_sender = SimpleSender()
    
    print("Keyboard Teleop: Use 'z' (forward), 's' (backward), 'q' (left), 'd' (right). Press 'x' to exit.")
    
    try:
        while rclpy.ok():
            key = getch()
            
            if key == 'z':  # Move forward
                message = "04000400\n\r\0"
            elif key == 's':  # Move backward
                message = "00000000\n\r\0"
            elif key == 'q':  # Turn left
                message = "04000100\n\r\0"
            elif key == 'd':  # Turn right
                message = "01000400\n\r\0"
            elif key == 'x':  # Exit
                break
            else:
                continue
            
            simple_sender.send_serial_message(message)  
            rclpy.spin_once(simple_sender, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
