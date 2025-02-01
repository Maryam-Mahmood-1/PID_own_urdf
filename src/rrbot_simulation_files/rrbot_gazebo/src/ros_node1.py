#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

class PositionPublisherNode(Node):
    def __init__(self):
        super().__init__('position_publisher_node')
        self.serial_port = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz timer
        self.reference_position1 = 0.0  # Initial reference position 1
        self.reference_position2 = 0.0  # Initial reference position 2

    def send_position(self):
        packet = f'<P,{self.reference_position1:.2f},{self.reference_position2:.2f}>'
        self.serial_port.write((packet + '\n').encode('utf-8'))
        self.get_logger().info(f'Sent reference positions: {self.reference_position1:.2f}, {self.reference_position2:.2f}')

    def timer_callback(self):
        # Simulate incrementing reference positions (you can replace this with your own logic)
        self.reference_position1 += 1.0
        self.reference_position2 += 2.0
        if self.reference_position1 > 10.0:
            self.reference_position1 = 0.0  # Reset reference position for testing
        if self.reference_position2 > 10.0:
            self.reference_position2 = 0.0  # Reset reference position for testing
        self.send_position()

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
