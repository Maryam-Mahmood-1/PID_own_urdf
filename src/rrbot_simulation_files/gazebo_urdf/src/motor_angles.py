#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class MotorControlPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motor_angles', 10)

        # Default angles for Motors 1 through 4 in degrees
        self.angles_deg = [0.0] * 4  # Store all angles in a list

    def publish_motor_angles(self):
        """Publish the current motor angles to the ROS topic in radians."""
        # Convert angles to radians
        angles_rad = [math.radians(angle) for angle in self.angles_deg]

        # Publish the angles in radians
        msg = Float64MultiArray()
        msg.data = angles_rad
        self.publisher_.publish(msg)

        # Log the published angles
        self.get_logger().info(f"Published angles (radians): {angles_rad}")

    def set_angles(self, *angles):
        """Set the angles for the motors and trigger the necessary actions."""
        if len(angles) != 4:
            self.get_logger().error("Expected 4 angles, but got fewer or more.")
            return

        self.angles_deg = list(angles)

        # Publish the angles to the ROS topic
        self.publish_motor_angles()

def main(args=None):
    rclpy.init(args=args)
    motor_control_publisher = MotorControlPublisher()

    try:
        while rclpy.ok():
            # Prompt the user for input to update Motor 1 through Motor 4 angles
            user_input = input("Enter angles for Motors 1 through 4 (in degrees, separated by spaces): ")
            try:
                # Split the input into four values
                angles = list(map(float, user_input.split()))
                if len(angles) != 4:
                    raise ValueError("You must enter exactly 4 angles.")
                motor_control_publisher.set_angles(*angles)  # Update motor angles
            except ValueError as e:
                print(f"Invalid input: {e}. Please enter four numerical values separated by spaces.")

    except KeyboardInterrupt:
        pass

    motor_control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
