#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import serial
import numpy as np
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.num_joints = 4
        self.target_positions = [0.0] * self.num_joints
        self.current_positions = [0.0] * self.num_joints
        self.current_velocities = [0.0] * self.num_joints

        self.rad_target_positions = [0.0] * self.num_joints
        self.rad_current_positions = [0.0] * self.num_joints
        self.rad_current_velocities = [0.0] * self.num_joints

        self.proportional_gain = [1, 10, 5.88, 5]
        self.derivative_gain = [0.0, 0.2, 0.2, 0.2]
        self.integral_gain = [0.0, 0.0001, 0.0, 0.0]
        self.integral_error = [0.0] * self.num_joints
        self.i_clamp = [450] * self.num_joints
        self.error_threshold = 0.001  # Threshold for considering a target position reached
        self.setup_done = False
        self.setup_done_recv = False
        self.revc_from_arduino = False

        self.errors_publisher_ = self.create_publisher(Float64MultiArray, 'joint_errors', 10)
        self.errors_rad_publisher_ = self.create_publisher(Float64MultiArray, 'joint_errors_rad', 10)

        self.reference_publisher_ = self.create_publisher(Float64MultiArray, 'reference_positions', 10)
        self.setup_done_publisher_ = self.create_publisher(Bool, 'setup_done', 10)

        self.target_subscription_ = self.create_subscription(
            Float64MultiArray,
            'motor_angles',
            self.target_callback,
            10
        )

        self.serial_timer_ = self.create_timer(0.1, self.serial_communication_callback)

        # Initialize serial communication with Arduino
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1, write_timeout=1)

    def target_callback(self, msg):
        if self.setup_done:
            print("I am here")
            print("message = ")
            print(msg)
            self.target_positions = msg.data

    def serial_communication_callback(self):
        received_message = self.receive_from_arduino()
        if received_message:
            #print("Received a message")
            self.revc_from_arduino  = True
            self.parse_arduino_message(received_message)

    def receive_from_arduino(self):
        try:
            if self.serial_port.in_waiting > 0:
                message = self.serial_port.readline().decode('utf-8').strip()
                print("Arduino has sent message: ")
                print(message)
                return message
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        return None

    def parse_arduino_message(self, message):
        if message.startswith('!') and message.endswith('%'):
            message = message[1:-1]  # Remove '!' and '%'

            try:
                t_index = message.index("#T_")
                p_index = message.index("#P_")
                v_index = message.index("#V_")

                target_str = message[t_index + 3:p_index - 2]
                position_str = message[p_index + 3:v_index - 2]
                velocity_str = message[v_index + 3:]

                print(target_str)
                print(position_str)
                print(velocity_str)
                if not self.setup_done:
                    self.target_positions = list(map(float, target_str.split()))
                self.current_positions = list(map(float, position_str.split()))  # Errors are parsed but not stored globally.
                self.current_velocities = list(map(float, velocity_str.split()))

                self.get_logger().info(f"Parsed from Arduino: Targets={self.target_positions}, Velocities={self.current_velocities}")
                self.calculate_and_publish_errors()
            except ValueError as e:
                self.get_logger().error(f"Error parsing Arduino message: {e}")

    def calculate_and_publish_errors(self):
        # Convert target and current positions to radians
        self.rad_target_positions = (np.array(self.target_positions) / 180) * np.pi
        self.rad_current_positions = (np.array(self.current_positions) / 180) * np.pi
        self.rad_current_velocities = (np.array(self.current_velocities) / 180) * np.pi

        # Calculate errors in degrees and radians
        errors = [self.target_positions[i] - self.current_positions[i] for i in range(self.num_joints)]
        errors_e = [self.rad_target_positions[i] - self.rad_current_positions[i] for i in range(self.num_joints)]

        # Publish degree errors
        error_message = Float64MultiArray()
        error_message.data = errors
        self.errors_publisher_.publish(error_message)

        # Publish radian errors
        error_rad_message = Float64MultiArray()
        error_rad_message.data = errors_e
        self.errors_rad_publisher_.publish(error_rad_message)

        # Log the errors for debugging
        self.get_logger().info(f"Degree Errors: {errors}")
        self.get_logger().info(f"Radian Errors: {errors_e}")

        # Update integral errors
        for i in range(self.num_joints):
            print("here in the integral error loop")
            self.integral_error[i] += errors_e[i]
            self.integral_error[i] = np.clip(self.integral_error[i], -self.i_clamp[i], self.i_clamp[i])
        print("here after the integral error loop")
        # Calculate joint velocities
        apply_joint_velocities = []
        for i in range(self.num_joints):
            velocity_r = (
                -(self.proportional_gain[i] * errors_e[i])
                - (self.derivative_gain[i] * self.rad_current_velocities[i])
                - (self.integral_gain[i] * self.integral_error[i])
            )
            velocity = (velocity_r * 180) / np.pi
            apply_joint_velocities.append(velocity)
        print("here after calculating velocities")


        # Check setup completion
        if all(abs(e) < self.error_threshold for e in errors_e) and not self.setup_done_recv:
            self.setup_done = True
            self.setup_done_recv = True
            self.target_positions = [0.0] * self.num_joints
            self.current_positions = self.target_positions + errors
            setup_done_msg = Bool()
            setup_done_msg.data = self.setup_done
            self.setup_done_publisher_.publish(setup_done_msg)
        # Send calculated velocities to Arduino
        self.send_to_arduino(apply_joint_velocities)

        print("here after sending velocities to arduino")


    def send_to_arduino(self, velocities):
        try:
            # Round velocities to the nearest integer
            rounded_velocities = [round(v) for v in velocities]
            print("sending velocities = ")
            print(rounded_velocities)

            # Format target positions to two decimal places
            formatted_targets = [f"{t:.2f}" for t in self.target_positions]
            print("formatted targets = ")
            print(formatted_targets)


            # Construct the command string
            command = (
                f"@#T_{' '.join(formatted_targets)}$ "
                f"#V_{' '.join(map(str, rounded_velocities))}$ "
                f"#S_{int(self.setup_done)}&\n"
            )
            self.serial_port.write(command.encode('utf-8'))
            #time.sleep(0.1)
            #self.serial_port.reset_output_buffer()

            print("Here after writing to serial port")

            self.get_logger().info(f"Sent to Arduino: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Error sending to Arduino: {e}")



def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.serial_port.close()
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
