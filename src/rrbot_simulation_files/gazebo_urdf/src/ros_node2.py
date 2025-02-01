#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import random

class VelocityPublisherNode(Node):
    def __init__(self):
        super().__init__('velocity_publisher_node')
        self.serial_port = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz timer
        self.current_position1 = 0.0  # Placeholder for received position 1
        self.current_position2 = 0.0  # Placeholder for received position 2
        self.velocity1 = 0.0  # Placeholder for velocity 1
        self.velocity2 = 0.0  # Placeholder for velocity 2

    def send_velocity(self):
        packet = f'<V,{self.velocity1:.2f},{self.velocity2:.2f}>'
        self.serial_port.write((packet + '\n').encode('utf-8'))
        self.get_logger().info(f'Sent velocities: {self.velocity1:.2f}, {self.velocity2:.2f}')

    def parse_position_packet(self, packet):
        try:
            commaIndex1 = packet.index(',')
            commaIndex2 = packet.rindex(',')  # Use rindex for the last comma
            if packet[1] == 'R':
                position1 = float(packet[commaIndex1 + 1:commaIndex2])
                position2 = float(packet[commaIndex2 + 1:-1])  # Exclude the closing '>'
                self.current_position1 = position1
                self.current_position2 = position2
                self.get_logger().info(f'Received current positions: {self.current_position1}, {self.current_position2}')
        except ValueError as e:
            self.get_logger().error(f'Error parsing position packet: {e}')

    def read_serial(self):
        if self.serial_port.in_waiting:
            packet = self.serial_port.readline().decode('utf-8').strip()
            if packet.startswith('<') and packet.endswith('>'):
                self.parse_position_packet(packet)

    def timer_callback(self):
        # Simulate random velocities for now
        self.velocity1 = random.uniform(-10.0, 10.0)  # Replace with your logic
        self.velocity2 = random.uniform(-10.0, 10.0)  # Replace with your logic
        self.send_velocity()
        self.read_serial()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
