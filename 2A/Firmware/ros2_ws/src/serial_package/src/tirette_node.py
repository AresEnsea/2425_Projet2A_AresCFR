#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class TiretteNode(Node):
    def __init__(self):
        super().__init__('tirette_node')
        # pin 27
        self.tirette_pin = 27
        GPIO.setmode(GPIO.BCM)
        # Configurer le pin comme entrée avec pull-up interne
        GPIO.setup(self.tirette_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.publisher = self.create_publisher(Bool, 'tirette_value', 10)
        # Créer un timer pour vérifier l'état de la tirette toutes les 0.1s
        self.timer = self.create_timer(0.1, self.check_tirette)
        self.get_logger().info('Tirette node started')

    def check_tirette(self):
        # Lire l'état du GPIO (False si LOW, True si HIGH)
        tirette_state = GPIO.input(self.tirette_pin) == GPIO.HIGH
        # Créer un message Bool
        msg = Bool()
        msg.data = tirette_state
        # Publier l'état
        self.publisher.publish(msg)
        self.get_logger().info(f'Tirette state: {tirette_state}')

    def destroy_node(self):
        # Nettoyer les GPIO avant de quitter
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TiretteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
