#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MotorAnglesPublisher(Node):
    def __init__(self):
        super().__init__('motor_angles_publisher')  # Initialize the node
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motor_angles', 10)  # Create publisher
        self.timer = self.create_timer(1.0, self.publish_motor_angles)  # Timer to publish every second
        self.get_logger().info("Motor Angles Publisher Node has been started.")
        self.motor_angles = []

    def publish_motor_angles(self):
        if self.motor_angles:
            msg = Float64MultiArray()  # Create the message object
            msg.data = self.motor_angles  # Set the data for the message
            self.publisher_.publish(msg)  # Publish the message
            self.get_logger().info(f"Publishing motor angles: {self.motor_angles}")
        else:
            self.get_logger().warn("No motor angles set, waiting for input...")

    def get_motor_angles_input(self):
        # Ask the user for input
        user_input = input("Enter motor angles (space-separated) or 's' to stop: ")
        if user_input.lower() == 's':
            self.get_logger().info("'s' command received. Stopping input and exiting...")
            rclpy.shutdown()
            return

        try:
            # Parse input and store it as a list of float numbers
            self.motor_angles = [float(angle) for angle in user_input.split()]
            self.get_logger().info(f"Motor angles updated: {self.motor_angles}")
        except ValueError:
            self.get_logger().error("Invalid input! Please enter space-separated numeric values or 's' to stop.")

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    motor_angles_publisher = MotorAnglesPublisher()  # Create an instance of the node

    # Continuously take user input in the main loop and publish angles
    try:
        while rclpy.ok():
            motor_angles_publisher.get_motor_angles_input()  # Get motor angles from the user
            rclpy.spin_once(motor_angles_publisher)  # Handle any pending ROS 2 work
    except KeyboardInterrupt:
        motor_angles_publisher.get_logger().info("Keyboard interrupt received. Shutting down...")

    motor_angles_publisher.destroy_node()  # Destroy the node once done
    rclpy.shutdown()  # Shut down the rclpy library

if __name__ == '__main__':
    main()





'''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MotorAnglesPublisher(Node):
    def __init__(self):
        super().__init__('motor_angles_publisher')  # Initialize the node
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motor_angles', 10)  # Create publisher
        self.timer = self.create_timer(1.0, self.publish_motor_angles)  # Timer to publish every second
        self.get_logger().info("Motor Angles Publisher Node has been started.")
        self.motor_angles = [None] * 7  # Initialize 7 motors with None (to represent stopped motors)

    def publish_motor_angles(self):
        if any(angle is not None for angle in self.motor_angles):
            msg = Float64MultiArray()  # Create the message object
            msg.data = [angle if angle is not None else float('nan') for angle in self.motor_angles]  # Convert None to NaN
            self.publisher_.publish(msg)  # Publish the message
            self.get_logger().info(f"Publishing motor angles: {self.motor_angles}")
        else:
            self.get_logger().warn("No motor angles set, waiting for input...")

    def get_motor_angles_input(self):
        user_input = input("Enter angles for 7 motors (space-separated, 's' to stop a motor): ").strip().split()

        if len(user_input) != 7:
            self.get_logger().error("Invalid input! Please enter exactly 7 values (either floats or 's').")
            return

        for i, value in enumerate(user_input):
            if value.lower() == 's':
                self.motor_angles[i] = None  # Motor is stopped (None represents stop)
                self.get_logger().info(f"Motor {i + 1} is stopped.")
            else:
                try:
                    self.motor_angles[i] = float(value)  # Parse the input as a float
                    self.get_logger().info(f"Motor {i + 1} angle set to {self.motor_angles[i]}")
                except ValueError:
                    self.get_logger().error(f"Invalid input for motor {i + 1}! Please enter a numeric value or 's' to stop.")
                    return

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    motor_angles_publisher = MotorAnglesPublisher()  # Create an instance of the node

    # Continuously take user input in the main loop and publish angles
    try:
        while rclpy.ok():
            motor_angles_publisher.get_motor_angles_input()  # Get motor angles from the user
            rclpy.spin_once(motor_angles_publisher)  # Handle any pending ROS 2 work
    except KeyboardInterrupt:
        motor_angles_publisher.get_logger().info("Keyboard interrupt received. Shutting down...")

    motor_angles_publisher.destroy_node()  # Destroy the node once done
    rclpy.shutdown()  # Shut down the rclpy library

if __name__ == '__main__':
    main()
'''







'''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MotorAnglesPublisher(Node):
    def __init__(self):
        super().__init__('motor_angles_publisher')  # Initialize the node
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motor_angles', 10)  # Create publisher
        self.timer = self.create_timer(1.0, self.publish_motor_angles)  # Timer to publish every second
        self.get_logger().info("Motor Angles Publisher Node has been started.")
        self.motor_angles = []

    def publish_motor_angles(self):
        if self.motor_angles:
            msg = Float64MultiArray()  # Create the message object
            msg.data = self.motor_angles  # Set the data for the message
            self.publisher_.publish(msg)  # Publish the message
            self.get_logger().info(f"Publishing motor angles: {self.motor_angles}")
        else:
            self.get_logger().warn("No motor angles set, waiting for input...")

    def get_motor_angles_input(self):
        # Ask the user for input
        user_input = input("Enter motor angles (space-separated): ")
        try:
            # Parse input and store it as a list of float numbers
            self.motor_angles = [float(angle) for angle in user_input.split()]
            self.get_logger().info(f"Motor angles updated: {self.motor_angles}")
        except ValueError:
            self.get_logger().error("Invalid input! Please enter space-separated numeric values.")

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    motor_angles_publisher = MotorAnglesPublisher()  # Create an instance of the node

    # Continuously take user input in the main loop and publish angles
    while rclpy.ok():
        motor_angles_publisher.get_motor_angles_input()  # Get motor angles from the user
        rclpy.spin_once(motor_angles_publisher)  # Handle any pending ROS 2 work

    motor_angles_publisher.destroy_node()  # Destroy the node once done
    rclpy.shutdown()  # Shut down the rclpy library

if __name__ == '__main__':
    main()
'''
