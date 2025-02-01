#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import math

class MotorControlPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motor_angles', 10)

        # Initialize serial connection
        self.serial_port = '/dev/ttyACM0'  # Replace with your Arduino serial port
        self.baud_rate = 115200
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        self.angle1_deg = 0.0  # Default angle for Motor 1 in degrees
        self.angle2_deg = 0.0  # Default angle for Motor 2 in degrees

    def send_data_to_arduino(self):
        """Send the current motor angles to the Arduino."""
        # Format the message according to the specified protocol
        message = f"#{self.angle1_deg} {self.angle2_deg}$\n"
        self.serial_connection.write(message.encode('utf-8'))
        self.get_logger().info(f'Sent formatted message to Arduino: {message.strip()}')

    def publish_motor_angles(self):
        """Publish the current motor angles to the ROS topic in radians."""
        # Convert angles to radians
        angle1_rad = math.radians(self.angle1_deg)
        angle2_rad = math.radians(self.angle2_deg)

        # Publish the angles in radians
        msg = Float64MultiArray()
        msg.data = [angle1_rad, angle2_rad]
        self.publisher_.publish(msg)

        # Log the published angles
        self.get_logger().info(f"Published angles (radians): {angle1_rad}, {angle2_rad}")

    def set_angles(self, angle1, angle2):
        """Set the angles for both motors and trigger the necessary actions."""
        self.angle1_deg = angle1
        self.angle2_deg = angle2

        # Send data to Arduino
        self.send_data_to_arduino()

        # Publish the angles to the ROS topic
        self.publish_motor_angles()

def main(args=None):
    rclpy.init(args=args)
    motor_control_publisher = MotorControlPublisher()

    try:
        while rclpy.ok():
            # Prompt the user for input to update Motor 1 and Motor 2 angles
            user_input = input("Enter angles for Motor 1 and Motor 2 (in degrees, separated by space): ")
            try:
                # Split the input into two values
                angle1, angle2 = map(float, user_input.split())
                motor_control_publisher.set_angles(angle1, angle2)  # Update motor angles
            except ValueError:
                print("Invalid input. Please enter two numerical values separated by space.")

    except KeyboardInterrupt:
        pass

    motor_control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



'''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_angle_input', 10)
        
    def get_user_input(self):
        while True:
            try:
                input_str = input("Enter joint angles in degrees separated by space (e.g., '90 45'): ")
                # Convert degrees to radians
                angles_degrees = list(map(float, input_str.split()))
                if len(angles_degrees) != 2:
                    self.get_logger().info("Please enter exactly two angles.")
                    continue
                angles_radians = [math.radians(angle) for angle in angles_degrees]
                self.publish_joint_angles(angles_radians)
            except ValueError:
                self.get_logger().info("Invalid input. Please enter valid numeric values.")

    def publish_joint_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joint angles in radians: {angles}")

def main(args=None):
    rclpy.init(args=args)
    node = UserInputNode()
    node.get_logger().info("User Input Node: Ready to take input in degrees.")
    node.get_user_input()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
'''