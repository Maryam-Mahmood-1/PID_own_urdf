#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import numpy as np

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Global variables
        self.num_joints = 7
        self.initial_positions = [0] * self.num_joints
        self.current_positions = [0] * self.num_joints
        self.target_positions = [0] * self.num_joints
        self.offset_positions = [0] * self.num_joints
        self.required_speeds = [0] * self.num_joints
        self.current_speeds = [0] * self.num_joints
        self.position_constraints = [(-120, 120)] * self.num_joints  # example constraints in degrees
        self.initial_angles = [0] * self.num_joints  # example initialization values

        # PID parameters
        self.proportional_gain = [3.88, 3.88, 0.5, 3.88, 2.00, 2.00, 2.00]
        #                        [141,   142,   144,  143,  146,  145,  147]
        self.derivative_gain = [0.0, 0.2, 0.003, 0.2, 0.2, 0.2, 0.2]
        self.integral_gain = [0.000196, 0.000196, 0.0003, 0.000196, 0.000196, 0.000196, 0.000196]
        self.integral_error = [0.0] * self.num_joints
        self.integral_error_r = [0.0] * self.num_joints
        self.i_clamp = [300] * self.num_joints  # Clamp integral errors to a fixed range
        self.apply_joint_velocities = [0] * self.num_joints
        self.joint_reached = [False] * self.num_joints
        self.acceptable_error = 0.0001
        self.poweroff = True
        self.stoppedam = False
        self.stopm = [False, False, False, False, False, False, False]
        self.calc_vel_rad = [0] * self.num_joints

        # Serial communication setup
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        #self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)

        # Subscriber for receiving target angles
        self.create_subscription(
            Float64MultiArray,
            'motor_angles',
            self.motor_angles_callback,
            10
        )

        # Publishers
        self.errors_publisher_ = self.create_publisher(Float64MultiArray, 'joint_errors', 10)
        self.velocities_publisher_ = self.create_publisher(Float64MultiArray, 'joint_velocities', 10)
        self.reference_value_publisher_ = self.create_publisher(Float64MultiArray, 'reference_joint_states', 10)
        self.current_state_publisher_ = self.create_publisher(Float64MultiArray, 'current_joint_states', 10)

        # Setup sequence
        self.clear_serial_buffer()
        #self.read_motor_voltage()
        self.setup_serial_communication()

    def setup_serial_communication(self):
        self.turn_relay1_off()
        self.turn_relay2_off()
        while self.poweroff:
            time.sleep(0.1)
            self.get_logger().info("Powered off state")
            self.read_motor_voltage()

        if not self.poweroff:
            self.clear_serial_buffer()
            self.read_initial_motor_positions()
            self.calculate_pid_speeds()
            self.turn_relay1_on()
            self.turn_relay2_on()

    def clear_serial_buffer(self):
        # Send the clear buffer command
        self.serial_port.write(b"csb\n")
        self.get_logger().info("Clearing serial buffer...")

        # Wait for acknowledgment of clearing (i.e., <Cleared>)
        while True:
            response = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Raw received response: {response}")
            if response == "<Cleared>":
                self.get_logger().info("Buffer cleared successfully.")
                break  # Exit the loop once we receive the correct acknowledgment
            elif response:
                #pass
                self.get_logger().info(f"Unexpected response: {response}")
            time.sleep(0.001)  # Optional small delay to avoid busy-waiting
    
    def read_motor_voltage(self):
        # Send the read motor voltage command
        self.serial_port.write(b"rmv\n")
        self.get_logger().info("Requesting Motor Voltage...")

        attempts = 0  # Counter for the number of attempts
        max_attempts = 3  # Maximum number of attempts

        while attempts < max_attempts:
            response = self.serial_port.readline().decode().strip()

            if response.startswith("<a") and response.endswith(">"):
                # Extract the voltage value from the response
                try:
                    voltage = float(response[2:-1])  # Extract and convert to float
                    self.get_logger().info(f"Motor Voltage: {voltage}V")
                    if voltage < 45:
                        self.poweroff = True
                        self.setup_serial_communication()
                    else:
                        self.poweroff = False
                    return voltage  # Return the voltage value if needed
                except ValueError:
                    self.get_logger().error(f"Failed to parse voltage from response: {response}")
            elif response == "Error reading voltage":
                self.get_logger().error("Error received from Arduino while reading voltage.")
                return None
            elif response:
                self.get_logger().info(f"Unexpected response: {response}")

            attempts += 1  # Increment the attempt counter
            time.sleep(0.001)  # Small delay to avoid busy-waiting

        # If we reach here, all attempts failed
        self.get_logger().error("Failed to read motor voltage after 3 attempts. Voltage value unchanged.")
        return None



    def read_initial_motor_positions(self):
        while True:
            # Send the command every time to get a new response
            self.serial_port.write(b"rmpv\n")
            self.get_logger().info("Requesting initial motor positions...")

            response = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Initial motor positions response: {response}")

            # Check if the response starts with "<P", contains "V", and ends with ">"
            if response.startswith("<P") and "V" in response and response.endswith(">"):
                parsed_data = self.parse_arduino_response(response)

                # Check for None values in positions or speeds, indicating errors
                if any(p is None for p in parsed_data["positions"]) or any(s is None for s in parsed_data["speeds"]):
                    self.get_logger().warn("Error reading motor positions or speeds, retrying...")
                    time.sleep(0.001)  # Optional delay before retrying
                    continue  # Retry reading the positions and speeds, resend the command

                # If valid data is found, update positions and speeds
                self.initial_positions = parsed_data["positions"]
                self.target_positions = parsed_data["positions"]
                self.current_positions = parsed_data["positions"]
                self.current_speeds = parsed_data["speeds"]

                self.get_logger().info("Successfully read initial motor positions and speeds.")
                break  # Exit the loop once data is successfully read

            time.sleep(0.0001)  # Optional small delay to prevent busy-waiting


    def write_motor_positions_with_pid_speeds(self):
        # Prepare the command to write motor positions and speeds
        command = f"wmpv<P{' '.join(map(str, self.target_positions))} V{' '.join(map(str, self.apply_joint_velocities))}>\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Writing motor positions with PID speeds: {command.strip()}")
        start_time = time.time()
        
        while True:
            # Wait for the expected response: "<Written>" or an error message
            response = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Write motor positions with PID speeds response: {response}")
            self.get_logger().info(f"Response type: {type(response)}")

            if response == "<Written>":
                elapsed_time = time.time() - start_time
                self.get_logger().info(f"Time spent waiting for '<Written>': {elapsed_time:.6f} seconds")
                self.get_logger().info("Successfully wrote motor positions and speeds.")
                break  # Exit the loop once the correct response is received

            # Handle motor-specific error response
            elif response.startswith("Err") and response.endswith("-"):
                try:
                    motor_index = int(response[3:-1])  # Extract motor index from the error message
                    self.get_logger().error(f"Motor {motor_index} reported an error: {response}")
                    
                    # Placeholder for handling specific motor errors
                    # Add your functionality here (e.g., retry logic, logging, stopping the motor, etc.)
                    self.stop_motor(motor_index)
                except ValueError:
                    self.get_logger().error(f"Failed to parse motor index from response: {response}")
            
            # If unexpected response, log it and retry
            elif response:
                self.get_logger().warn(f"Unexpected response: {response}, retrying...")
            
            time.sleep(0.0001)  # Optional small delay to prevent busy-waiting

    def stop_motor(self, motor_index):
        """
        Sends a stop command to the specified motor.
        
        Args:
            motor_index (int): The index of the motor to stop (1-7).
        """
        if 1 <= motor_index <= 7:  # Ensure motor index is valid
            command = f"stop{motor_index}\n"  # Construct the stop command
            self.serial_port.write(command.encode())  # Send the command
            self.get_logger().info(f"Sent stop command to motor {motor_index}: {command.strip()}")
            
            # Wait for acknowledgment or log response
            response = self.serial_port.readline().decode().strip()
            #self.get_logger().info(f"Response for stopping motor {motor_index}: {response}")
            
            if response == f"<Stopped{motor_index}>":
                self.get_logger().info(f"Motor {motor_index} successfully stopped.")
            else:
                self.get_logger().warn(f"Unexpected response for motor {motor_index}: {response}")
        else:
            self.get_logger().error(f"Invalid motor index: {motor_index}. Must be between 1 and 7.")

    
    def stop_all_motors(self):
        """
        Sends a stop command to the specified motor.
        
        Args:
            motor_index (int): The index of the motor to stop (1-7).
        """
        if self.stoppedam == True:
            for motor_index in range(1, len(self.target_positions) + 1):  # Assuming motors are indexed 1 to n
                stop_command = f"stop{motor_index}\n"
                self.serial_port.write(stop_command.encode())
                self.get_logger().info(f"Sent stop command to motor {motor_index}")
        



        # Add your error-handling logic here


    def turn_relay1_on(self):
        self.serial_port.write(b"ron1\n")
        response = self.serial_port.readline().decode().strip()
        self.get_logger().info(f"Relay off response: {response}")

    def turn_relay2_on(self):
        self.serial_port.write(b"ron2\n")
        response = self.serial_port.readline().decode().strip()
        self.get_logger().info(f"Relay off response: {response}")
    
    def turn_relay1_off(self):
        self.serial_port.write(b"roff1\n")
        response = self.serial_port.readline().decode().strip()
        self.get_logger().info(f"Relay off response: {response}")
    
    def turn_relay2_off(self):
        self.serial_port.write(b"roff2\n")
        response = self.serial_port.readline().decode().strip()
        self.get_logger().info(f"Relay off response: {response}")

    def parse_arduino_response(self, response):
        try:
            response = response.strip('<>')  # Remove the surrounding angle brackets
            if response.startswith("P"):
                parts = response[1:].split("V")  # Split the positions and velocities
                positions = self.parse_motor_data(parts[0])  # Parse positions
                speeds = self.parse_motor_data(parts[1])  # Parse speeds
                return {"positions": positions, "speeds": speeds}
            elif response.startswith("a"):  # For voltage data, if needed
                return {"voltage": int(response[1:])}
            else:
                self.get_logger().error(f"Unknown response format: {response}")
        except Exception as e:
            self.get_logger().error(f"Error parsing response: {response} | {e}")
        return {}

    def parse_motor_data(self, data):
        """Helper function to parse motor data and handle 'E' values."""
        values = data.split()  # Split by spaces
        parsed_values = []

        for value in values:
            if value == 'E':
                parsed_values.append(None)  # Use None to represent an error
            else:
                try:
                    parsed_values.append(int(value))  # Convert to integer
                except ValueError:
                    parsed_values.append(None)  # In case there's another non-integer value
        return parsed_values


    def motor_angles_callback(self, msg):
        # Convert the data to a list of angles, handling None values for stopped motors
        motor_recv_angles = [
            max(min(int(angle), upper), lower) if angle is not None else None
            for angle, (lower, upper) in zip(msg.data, self.position_constraints)
        ]

        # Log the initial positions and the received angles
        self.get_logger().info(f"Initial positions were: {self.initial_positions}")
        self.get_logger().info(f"Received motor angles: {motor_recv_angles}")

        # Update target positions only if the angle is not None
        self.target_positions = []
        for a, angle in zip(self.initial_positions, motor_recv_angles):
            if angle is not None:
                self.target_positions.append(a + angle)  # Add the angle to the initial position
            else:
                self.stopm[self.initial_positions.index(a)] = True
                self.target_positions.append(a)  # Keep the initial position for the stopped motor


        # Log the converted target positions
        self.get_logger().info(f"Converted target positions: {self.target_positions}")

        # Call the PID calculation and motor writing methods
        self.calculate_pid_speeds()
        self.update_current_positions_and_speeds()




    def calculate_pid_speeds(self):
        error = [float(self.target_positions[i]) - float(self.current_positions[i]) for i in range(self.num_joints)]
        self.get_logger().info(f"Error = {error}")

        error_r = [(float(self.target_positions[i]) - float(self.current_positions[i]))/57.29 for i in range(self.num_joints)]
        
        # Publish joint errors
        error_message = Float64MultiArray()
        error_message.data = error  # Ensure that error values are of type float
        self.errors_publisher_.publish(error_message)
        self.get_logger().info(f"Publishing Errors in degrees (q1-q7): {error}")
        self.get_logger().info(f"Publishing Errors in radians (q1-q7): {error_r}")

        # Update integral error with clamping
        for i in range(self.num_joints):
            self.integral_error_r[i] += error_r[i]
            self.integral_error_r[i] = np.clip(self.integral_error[i], -self.i_clamp[i], self.i_clamp[i])

        # Calculate velocities (instead of efforts)
        for i in range(self.num_joints):
            if abs(error[i]) > self.acceptable_error:
                self.joint_reached[i] = False
                self.calc_vel_rad[i] = -(self.proportional_gain[i] * error_r[i]) \
                                                - (self.derivative_gain[i] * self.current_speeds[i]/57.29) \
                                                - (self.integral_gain[i] * self.integral_error_r[i])
                # Convert to dps
                self.apply_joint_velocities[i] = int(np.clip(self.calc_vel_rad[i]*57.29, -15, 15))
            else:
                self.joint_reached[i] = True
                self.apply_joint_velocities[i] = 0

        # Ensure that apply_joint_velocities are floats before publishing
        message = Float64MultiArray()
        message.data = [float(v) for v in self.apply_joint_velocities]  # Convert velocities to float
        self.velocities_publisher_.publish(message)

        # Publish reference joint states (target positions)
        reference_joint_states = Float64MultiArray()
        reference_joint_states.data = [float(v) for v in self.target_positions]  # Convert positions to float
        self.reference_value_publisher_.publish(reference_joint_states)

        # Publish current joint position and velocity (both positions and velocities need to be floats)
        current_state_message = Float64MultiArray()
        current_state_message.data = [float(p) for p in self.current_positions] + [float(v) for v in self.current_speeds]  # Convert both positions and velocities
        self.current_state_publisher_.publish(current_state_message)

        self.get_logger().info(f"Publishing Joint Velocities (v1-v7): {self.apply_joint_velocities}")
        
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {current_state_message.data}")
        self.get_logger().info(f"Publishing Integral Error: {self.integral_error_r}")
        self.get_logger().info(f"Joint Reached Status: {self.joint_reached}")
        self.get_logger().info(f"Error: {error}")



    def update_current_positions_and_speeds(self):
        max_retries = 300  # Maximum number of retries
        retries = 0  # Retry counter

        if self.stoppedam == True:
            for motor_index in range(len(self.target_positions)):  # Assuming motors are indexed 1 to n
                self.current_positions[motor_index] = self.current_positions[motor_index]
                self.current_speeds[motor_index] = 0

        # Prepare the command to write motor positions and speeds
        command = f"wmpv<P{' '.join(map(str, self.target_positions))} V{' '.join(map(str, self.apply_joint_velocities))}>\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Writing motor positions with PID speeds: {command.strip()}")
        #start_time = time.time()
        

        while retries < max_retries:
            # Send the command to request positions and speeds
            command = f"wmpv<P{' '.join(map(str, self.target_positions))} V{' '.join(map(str, self.apply_joint_velocities))}>\n"
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Writing motor positions with PID speeds: {command.strip()}")

            response = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Updating current motor positions and speeds: {response}")

            # Check if the response is in the expected format
            if response.startswith("<_") and response.endswith("_>"):
                try:
                    # Parse the motor data from the response
                    parsed_data = self.parse_motor_p_v(response)

                    # Check for errors in the parsed data
                    for motor_id, data in parsed_data.items():
                        if data["error"]:
                            self.get_logger().warn(f"Error reading motor {motor_id} data, retrying...")
                            retries += 1  # Increment retry counter
                            time.sleep(0.001)  # Optional delay before retrying
                            break  # Retry the entire process for errors
                    else:
                        # Update only the motors mentioned in the response
                        for motor_id, data in parsed_data.items():
                            if self.stopm[motor_id - 1] == False:
                                self.current_positions[motor_id - 1] = data["angle"] if data["angle"] is not None else self.current_positions[motor_id - 1]
                                self.current_speeds[motor_id - 1] = data["speed"] if data["speed"] is not None else 0
                            else:
                                self.current_positions[motor_id - 1] = self.current_positions[motor_id - 1]
                                self.current_speeds[motor_id - 1] = 0


                        self.get_logger().info("Successfully read current motor positions and speeds.")
                        return  # Exit the method on success

                except Exception as e:
                    self.get_logger().error(f"Error parsing motor data: {e}")
                    retries += 1  # Increment retry counter
                    time.sleep(0.01)  # Optional delay before retrying
                    continue

            # If response is invalid, increment retries and log a warning
            self.get_logger().warn("Invalid response, retrying...")
            retries += 1
            time.sleep(0.01)  # Optional delay before retrying


        # If retries are exhausted, stop all motors
        self.get_logger().error("Failed to read motor positions and speeds after maximum retries. Stopping all motors.")
        self.stoppedam = True
        self.stop_all_motors()
        self.setup_serial_communication()  # Reinitialize the serial communication


    def parse_motor_p_v(response):
        """
        Parses motor data from the response string.

        :param response: The response string (e.g., "<_3-1000-100_5-2000-200_7-E-E_>").
        :return: A dictionary with parsed motor data.
                Example:
                {
                    3: {"angle": 1000, "speed": 100, "error": False},
                    5: {"angle": 2000, "speed": 200, "error": False},
                    7: {"angle": None, "speed": None, "error": True}
                }
        """
        if not response.startswith("<_") or not response.endswith("_>"):
            raise ValueError(f"Invalid data format: {response}")

        # Strip the "<_" and "_>" wrappers
        raw_data = response[2:-2]

        # Parse motor data
        motor_data = {}
        motor_entries = raw_data.split("_")
        for entry in motor_entries:
            parts = entry.split("-")
            if len(parts) != 3:
                raise ValueError(f"Invalid motor entry: {entry}")

            motor_id = int(parts[0])
            angle = parts[1]
            speed = parts[2]

            # Handle "E" for errors
            if angle == "E" or speed == "E":
                motor_data[motor_id] = {"angle": None, "speed": None, "error": True}
            else:
                motor_data[motor_id] = {"angle": int(angle), "speed": int(speed), "error": False}

        return motor_data


    def timer_callback(self):
        if not self.poweroff:

            joint_displacements = [current - initial for current, initial in zip(self.current_positions, self.initial_positions)]
            self.get_logger().info(f"Joint displacements: {joint_displacements}")
            self.get_logger().info(f"Initial positions: {self.initial_positions}")
            self.calculate_pid_speeds()
            self.get_logger().info(f"Done calculating the required motors' speeds")
            self.update_current_positions_and_speeds()
    
    def voltage_callback(self):
        self.read_motor_voltage()


    def start_control_loop(self):
        # Timer for periodic updates (e.g., 10 Hz)
        self.timer = self.create_timer(0.0001, self.timer_callback)

    def voltage_checker(self):
        # Timer for periodic updates (e.g., 10 Hz)
        self.timer = self.create_timer(0.01, self.voltage_callback)

def main(args=None):
    rclpy.init(args=args)

    try:
        motor_controller = MotorController()
        motor_controller.start_control_loop()
        motor_controller.voltage_checker()

        rclpy.spin(motor_controller)
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        motor_controller.serial_port.close()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
