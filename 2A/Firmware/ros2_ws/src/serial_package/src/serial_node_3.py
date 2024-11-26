#!/usr/bin/env python3

import serial
import time
import keyboard  # Library to capture keypresses

class SimpleSender:
    def __init__(self):
        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

    def send_serial_message(self, message):
        try:
            ser = serial.Serial(self.uart2_port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return

        # Write message to serial port
        print(f"Sending message: {message.strip()}")
        ser.write(message.encode())
        ser.close()

    def control_robot(self):
        while True:
            if keyboard.is_pressed('z'):  # Move forward
                message = "04000400\n\r\0"
                self.send_serial_message(message)
            elif keyboard.is_pressed('s'):  # Stop
                message = "00000000\n\r\0"
                self.send_serial_message(message)
            elif keyboard.is_pressed('q'):  # Turn left
                message = "04000800\n\r\0"
                self.send_serial_message(message)
            elif keyboard.is_pressed('d'):  # Turn right
                message = "08000400\n\r\0"
                self.send_serial_message(message)
            time.sleep(0.1)  # Add a small delay to prevent excessive CPU usage

def main():
    sender = SimpleSender()
    sender.control_robot()

if __name__ == '__main__':
    main()
