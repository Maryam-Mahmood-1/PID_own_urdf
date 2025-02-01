#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import os
import serial  # Import the pyserial library

class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        # Initialize serial communication
        self.arduino_serial = serial.Serial('/dev/ttyACM1', 31250, timeout=1)  # Update port as needed
        self.get_logger().info("Serial connection to Arduino initialized.")

        # Subscriber to listen to joint angle input
        self.joint_angle_subscriber_ = self.create_subscription(
            Float64MultiArray, '/joint_angle_input', self.receive_joint_angles, 10
        )
        
        # Subscriber to listen to joint states from the robot
        self.joint_state_subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.calculate_joint_velocities, 10
        )
        
        # Publishers for joint velocities, reference joint states, and current state (position & velocity)
        self.velocities_publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.reference_value_publisher_ = self.create_publisher(Float64MultiArray, '/reference_joint_states/commands', 10)
        self.current_state_publisher_ = self.create_publisher(Float64MultiArray, '/current_joint_state', 10)

        # Initial values
        self.command_received_ = False
        self.reference_position = [0.0, 0.0]  # Default target position at start
        self.apply_joint_velocities = [0.0, 0.0]
        self.acceptable_error = 0.0001
        self.proportional_gain = [2.2588, 2.2588]
        self.derivative_gain = [0.2, 0.2]
        self.integral_gain = [0.0002, 0.0002]
        self.integral_error = [0.0, 0.0]
        self.i_clamp = [1.0, 450.0]

        # Publish default reference position and start control loop
        self.publish_default_reference_position()
        self.timer_ = self.create_timer(0.05, self.control_loop)

        # Timer for sending data to Arduino every 50ms
        self.arduino_timer_ = self.create_timer(0.1, self.send_data_to_arduino)

        # Current state message to send to Arduino
        self.current_state_message = Float64MultiArray()

    def publish_default_reference_position(self):
        """Publish the default reference position to ensure the robot starts at (0, 0)."""
        reference_joint_states = Float64MultiArray()
        reference_joint_states.data = self.reference_position
        self.reference_value_publisher_.publish(reference_joint_states)
        self.get_logger().info(f"Default reference position set to: {self.reference_position}")

    def control_loop(self):
        """Ensure robot moves to default position if no commands are received."""
        if not self.command_received_:
            message = Float64MultiArray()
            message.data = self.apply_joint_velocities
            self.velocities_publisher_.publish(message)
            self.get_logger().info("Control loop active: Ensuring robot is at (0, 0).")

    def receive_joint_angles(self, msg):
        # Receive joint angle input from the topic
        self.reference_position = msg.data  # Assuming msg.data contains [q1, q2]
        self.command_received_ = True
        self.get_logger().info(f"Received joint angles: {self.reference_position}")

    def calculate_joint_velocities(self, msg):
        joint_position = [msg.position[0], msg.position[1]]
        joint_velocity = [msg.velocity[0], msg.velocity[1]]
        error = [joint_position[0] - self.reference_position[0], joint_position[1] - self.reference_position[1]]

        # Update integral error with clamping
        self.integral_error[0] += error[0]
        self.integral_error[0] = np.clip(self.integral_error[0], -self.i_clamp[0], self.i_clamp[0])
        self.integral_error[1] += error[1]
        self.integral_error[1] = np.clip(self.integral_error[1], -self.i_clamp[1], self.i_clamp[1])

        # Calculate velocities (instead of efforts)
        if abs(error[0]) > self.acceptable_error:
            self.apply_joint_velocities[0] = -(self.proportional_gain[0] * error[0]) \
                                             - (self.derivative_gain[0] * joint_velocity[0]) \
                                             - (self.integral_gain[0] * self.integral_error[0])
        if abs(error[1]) > self.acceptable_error:
            self.apply_joint_velocities[1] = -(self.proportional_gain[1] * error[1]) \
                                             - (self.derivative_gain[1] * joint_velocity[1]) \
                                             - (self.integral_gain[1] * self.integral_error[1])

        # Publish joint velocities
        message = Float64MultiArray()
        message.data = self.apply_joint_velocities
        self.velocities_publisher_.publish(message)

        # Publish reference joint states
        reference_joint_states = Float64MultiArray()
        reference_joint_states.data = self.reference_position
        self.reference_value_publisher_.publish(reference_joint_states)

        # Update the current state message with position and velocity
        self.current_state_message.data = joint_position + joint_velocity

        # Publish current joint position and velocity
        self.current_state_publisher_.publish(self.current_state_message)

        self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
        self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {self.current_state_message.data}")
        self.get_logger().info(f"Publishing Integral Error: ({self.integral_error[1]})")

    def send_data_to_arduino(self):
        """Send data to the Arduino every 50ms."""
        if self.command_received_:  # Ensure commands have been received before sending
            try:
                # Format the data to send to Arduino
                serial_data = f"<{self.current_state_message.data[0]:.2f}," \
                              f"{self.current_state_message.data[1]:.2f}," \
                              f"{self.current_state_message.data[2]:.2f}," \
                              f"{self.current_state_message.data[3]:.2f}>\n"
                self.arduino_serial.write(serial_data.encode('utf-8'))
                self.get_logger().info(f"Timer sent to Arduino: {serial_data.strip()}")
            except Exception as e:
                self.get_logger().error(f"Failed to send data to Arduino: {e}")

    def close_serial_connection(self):
        """Close the serial connection during shutdown."""
        if self.arduino_serial.is_open:
            self.arduino_serial.close()
            self.get_logger().info("Closed serial connection to Arduino.")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateController()
    node.get_logger().info("Starting Joint Velocity Control.")

    # Start external commands
    os.system("ros2 run rrbot_gazebo switch_vel")
    os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        node.close_serial_connection()
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
