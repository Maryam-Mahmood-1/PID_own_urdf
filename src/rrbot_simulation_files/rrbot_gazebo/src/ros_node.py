#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import random

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Serial configuration
        self.serial_port = serial.Serial('/dev/ttyACM1', 31250, timeout=1)
        self.clear_serial_buffer()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz timer
        self.current_position = 0.0  # Placeholder for received position
        self.velocity = 0.0  # Placeholder for velocity to send

    # Clear serial buffer
    def clear_serial_buffer(self):
        while self.serial_port.in_waiting:
            self.serial_port.read()  # Discard any incoming data
        self.get_logger().info("Serial buffer cleared")

    # Send velocity command to Arduino
    def send_velocity(self):
        packet = f'<V{self.velocity:.2f}>'
        self.serial_port.write((packet + '\n').encode('utf-8'))
        self.get_logger().info(f'Sent velocity: {self.velocity:.2f}')

    # Parse position packet received from Arduino
    def parse_position_packet(self, packet):
        try:
            if packet[1] == 'P':
                position = float(packet[2:-1])
                self.current_position = position
                self.get_logger().info(f'Received position: {self.current_position}')
        except ValueError as e:
            self.get_logger().error(f'Error parsing position packet: {e}')

    # Read and parse data from Arduino
    def read_serial(self):
        if self.serial_port.in_waiting:
            packet = self.serial_port.readline().decode('utf-8').strip()
            if packet.startswith('<') and packet.endswith('>'):
                self.parse_position_packet(packet)

    # Timer callback to send and receive data
    def timer_callback(self):
        # Simulate random velocity for now
        self.velocity = random.uniform(-10.0, 10.0)  # Replace with your logic
        self.send_velocity()
        self.read_serial()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
