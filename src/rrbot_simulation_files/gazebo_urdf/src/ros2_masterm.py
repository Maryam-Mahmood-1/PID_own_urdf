#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import serial
import time
import numpy as np
import copy

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
        self.prev_motor_speeds = [0] * self.num_joints
        self.position_constraints = [(-120, 120)] * self.num_joints  # example constraints in degrees
        self.initial_angles = [0] * self.num_joints  # example initialization values

        # PID parameters
        self.proportional_gain = [0.6, 0.5, 2.0, 1.5, 3.0, 3.0, 3.5]
        #                        [141,   142,   144,  143,  146,   145,   147]
        self.derivative_gain = [0.003, 0.003, 0.000, 0.001, 0.002, 0.002, 0.0015]
        self.integral_gain = [0.000196, 0.000196, 0.0000, 0.0000, 0.0, 0.0, 0.0]
        self.integral_error = [0.0] * self.num_joints
        self.integral_error_r = [0.0] * self.num_joints
        self.dt = 0.06
        self.prev_PID_time = 0.0
        self.prev_error = [0.0] * self.num_joints
        self.prev_speed = [0.0] * self.num_joints
        self.alpha = 0.9
        self.lpfw = 0.5
        self.m_lpfw = 0.5
        self.lpfw_dr = 0.87
        self.lpfw_dr_f = 0
        self.i_clamp = [600] * self.num_joints  # Clamp integral errors to a fixed range
        self.apply_joint_velocities = [0] * self.num_joints
        self.write_velocities = [True] * self.num_joints
        self.joint_reached = [False] * self.num_joints
        self.acceptable_error = 0.0001
        self.poweroff = True
        self.calc_vel_rad = [0] * self.num_joints
        self.calc_vel_rad_with_speed = [0] * self.num_joints
        self.calc_vel_rad_with_motor_speed = [0] * self.num_joints
        self.current_calc_speed = [0] * self.num_joints
        self.current_calc_speed_d_speed = [0] * self.num_joints
        self.current_calc_speed_dm_speed = [0] * self.num_joints

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
        self.rel_value_publisher_ = self.create_publisher(Float64MultiArray, 'rel_joint_states', 10)
        self.org_value_publisher_ = self.create_publisher(Float64MultiArray, 'org_joint_states', 10)
        self.current_state_publisher_ = self.create_publisher(Float64MultiArray, 'current_joint_states', 10)
        self.D_error_publisher_ = self.create_publisher(Float64MultiArray, 'd_error', 10)
        self.Derivative_error_publisher_ = self.create_publisher(Float64MultiArray, 'derivative_error', 10)
        self.Derivativem_error_publisher_ = self.create_publisher(Float64MultiArray, 'mderivative_error', 10)
        self.vel_error_d = self.create_publisher(Float64MultiArray, 'speed_error_d', 10)
        self.vel_calc_speed = self.create_publisher(Float64MultiArray, 'speed_sim_vel', 10)
        self.vel_motor_speed = self.create_publisher(Float64MultiArray, 'speed_motor_vel', 10)
        self.weight_publisher_ = self.create_publisher(Float64, 'weight_filter', 10)
        self.dt_publisher_ = self.create_publisher(Float64, 'dt_PID', 10)

        # Setup sequence
        self.clear_serial_buffer()
        #self.read_motor_voltage()
        self.setup_serial_communication()

    def setup_serial_communication(self):
        self.turn_relay_off()
        while self.poweroff:
            time.sleep(0.1)
            self.get_logger().info("Powered off state")
            self.read_motor_voltage()

        if not self.poweroff:
            self.clear_serial_buffer()
            self.read_initial_motor_positions()
            self.calculate_pid_speeds()
            self.turn_relay_on()

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
                

                # Ensure deep copies are made to avoid shared references
                self.initial_positions = copy.deepcopy(parsed_data["positions"])
                self.target_positions = copy.deepcopy(parsed_data["positions"])
                self.current_positions = copy.deepcopy(parsed_data["positions"])
                self.current_speeds = copy.deepcopy(parsed_data["speeds"])

                self.get_logger().info("Successfully read initial motor positions and speeds.")
                break  # Exit the loop once data is successfully read

            time.sleep(0.0001)  # Optional small delay to prevent busy-waiting


    def write_motor_positions_with_pid_speeds(self):
        max_retries = 3  # Maximum number of retries
        retries = 0  # Retry counter
        self.prev_motor_speeds = self.current_speeds
        s_time = time.time()
        command = f"wmpv<V{' '.join(map(str, self.apply_joint_velocities))}>\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Writing motor positions with PID speeds: {command.strip()}")
        start_time = time.time()
        while True:
        # Prepare the command to write motor positions and speeds
        
            # Wait for the expected response: "<Written>" or an error message
            response = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Write motor positions with PID speeds response: {response}")
            self.get_logger().info(f"Response type: {type(response)}")

            if response.startswith("<P") and response.endswith(">"):
                parsed_data = self.parse_arduino_msg(response)  # Parse positions and speeds

                # Ensure parsed_data contains both positions and speeds
                if "positions" in parsed_data and "speeds" in parsed_data:
                    positions = parsed_data["positions"]
                    speeds = parsed_data["speeds"]

                    # Check for None values in positions, indicating errors
                    if any(p is None for p in positions):
                        self.get_logger().warn("Error reading motor positions, retrying...")
                        retries += 1  # Increment retry counter
                        time.sleep(0.0001)  # Optional delay before retrying
                        continue  # Retry reading the positions

                    # If no errors, store the parsed positions and speeds
                    
                    self.current_speeds = speeds  # Store speeds if needed for further logic
                else:
                    self.get_logger().error("Invalid data received from Arduino")

                # If valid data is found, update positions
                f_time = time.time()
                m_alpha = 1 - ((1 - self.lpfw) * (self.lpfw_dr ** self.lpfw_dr_f))  # Apply filtering

                for i in range(self.num_joints):
                    self.current_positions[i] = (
                        m_alpha * parsed_data["positions"][i] + (1 - m_alpha) * self.current_positions[i]
                    )

                self.get_logger().info(f"Passed time in reading positions: {f_time - s_time:.6f} seconds")
                self.get_logger().info("Successfully read current motor positions.")
                #self.prev_PID_time = s_time
                return  # Exit the method on success

            # If response is invalid, increment retries and log a warning
            self.get_logger().warn("Invalid response, retrying...")
            retries += 1
            time.sleep(0.00075)  # Optional delay before retrying
        
        

        # If retries are exhausted, stop all motors
        self.get_logger().error("Failed to read motor positions after maximum retries. Stopping all motors.")
        for motor_index in range(1, len(self.target_positions) + 1):  # Assuming motors are indexed 1 to n
            stop_command = f"stop{motor_index}\n"
            self.serial_port.write(stop_command.encode())
            self.get_logger().info(f"Sent stop command to motor {motor_index}")



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


        # Add your error-handling logic here


    def turn_relay_on(self):
        self.serial_port.write(b"ron\n")
        response = self.serial_port.readline().decode().strip()
        self.get_logger().info(f"Relay off response: {response}")
    
    def turn_relay_off(self):
        self.serial_port.write(b"roff\n")
        response = self.serial_port.readline().decode().strip()
        self.get_logger().info(f"Relay off response: {response}")

    def parse_arduino_msg(self, response):
        try:
            response = response.strip('<>')  # Remove angle brackets

            if response.startswith("P"):  # Check if it's a position/speed message
                parts = response[1:].split()  # Split values by space
                positions = []
                speeds = []

                for i, value in enumerate(parts):  # Iterate through the parsed parts
                    if value == "E,E":
                        positions.append(self.current_positions[i])  # Keep last known position
                        speeds.append(0)  # Speed unknown
                        self.write_velocities[i] = False
                    elif value == "Err,Err":
                        positions.append(None)  # Mark errors as None
                        speeds.append(0)
                    else:
                        pos, speed = value.split(",")  # Split into position and speed
                        positions.append(int(pos))  # Convert to integer
                        speeds.append(int(speed))

                return {"positions": positions, "speeds": speeds}

            elif response.startswith("a"):  # If it's voltage data
                return {"voltage": int(response[1:])}

            else:
                self.get_logger().error(f"Unknown response format: {response}")
        except Exception as e:
            self.get_logger().error(f"Error parsing response: {response} | {e}")

        return {}




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
        # Convert the data to a list of int32_t
        #motor_recv_angles = [int(angle) for angle in msg.data]
        motor_recv_angles = [
            max(min(int(angle), upper), lower) 
            for angle, (lower, upper) in zip(msg.data, self.position_constraints)
        ]

        self.get_logger().info(f"Initial positions were: {self.initial_positions}")
        self.target_positions = [a + b for a, b in zip(motor_recv_angles, self.initial_positions)]


        # Log the converted target positions
        self.get_logger().info(f"Converted target positions: {self.target_positions}")
        self.lpfw_dr_f = 0

        # Call the PID calculation and motor writing methods
        self.calculate_pid_speeds()
        self.write_motor_positions_with_pid_speeds()

    def calculate_pid_speeds(self):
        self.del_t = time.time() - self.prev_PID_time
        self.m_lpfw = 1 - ((1 - self.lpfw)*(self.lpfw_dr**self.lpfw_dr_f))
        msg = Float64()
        msg.data = self.m_lpfw  # Assign the filtered value
        self.weight_publisher_.publish(msg)

        msg = Float64()
        msg.data = self.del_t  # Assign the filtered value
        self.weight_publisher_.publish(msg)

        error = [float(self.target_positions[i]) - float(self.current_positions[i]) for i in range(self.num_joints)]
        self.get_logger().info(f"Error = {error}")

        error_r = [(float(self.target_positions[i]) - float(self.current_positions[i]))/57.29 for i in range(self.num_joints)]
        error_derivative = [(error_r[i] - self.prev_error[i]) / self.dt for i in range(self.num_joints)]
        self.prev_error = error_r
        
        # Publish joint errors
        error_message = Float64MultiArray()
        error_message.data = error  # Ensure that error values are of type float
        self.errors_publisher_.publish(error_message)
        self.get_logger().info(f"Publishing Errors in degrees (q1-q7): {error}")
        self.get_logger().info(f"Publishing Errors in radians (q1-q7): {error_r}")

        # Update integral error with clamping
        for i in range(self.num_joints):
            self.integral_error_r[i] += error_r[i] * self.dt
            self.integral_error_r[i] = np.clip(self.integral_error[i], -self.i_clamp[i], self.i_clamp[i])

        # Calculate velocities (instead of efforts)
        for i in range(self.num_joints):
            if abs(error[i]) > self.acceptable_error:
                self.joint_reached[i] = False
                self.calc_vel_rad[i] = (self.proportional_gain[i] * error_r[i]) \
                                                + (self.derivative_gain[i] * (error_derivative[i])) \
                                                + (self.integral_gain[i] * self.dt * self.integral_error_r[i])
                
                self.calc_vel_rad_with_speed[i] = (self.proportional_gain[i] * error_r[i]) \
                                                + (self.derivative_gain[i] * (-self.prev_speed[i])/57.29) \
                                                + (self.integral_gain[i] * self.dt * self.integral_error_r[i])
                
                self.calc_vel_rad_with_motor_speed[i] = (self.proportional_gain[i] * error_r[i]) \
                                                + (self.derivative_gain[i] * (-self.prev_motor_speeds[i])/57.29) \
                                                + (self.integral_gain[i] * self.dt * self.integral_error_r[i])


                # Convert to dps
                if abs(error[i]) > 2:
                    self.current_calc_speed[i] = int((np.clip(self.calc_vel_rad[i]*57.29, -60, 60))*100)
                
                else:
                    self.current_calc_speed[i] = int((np.clip(self.calc_vel_rad[i]*57.29, -60, 60))*100)
                
                self.current_calc_speed_d_speed[i] = int((np.clip(self.calc_vel_rad_with_speed[i]*57.29, -60, 60))*100)
                self.current_calc_speed_dm_speed[i] = int((np.clip(self.calc_vel_rad_with_motor_speed[i]*57.29, -60, 60))*100)
                if self.write_velocities[i] == True:
                    self.apply_joint_velocities[i] = int(self.m_lpfw * self.current_calc_speed[i] + (1 - self.m_lpfw) * self.prev_speed[i])
                else:
                    self.apply_joint_velocities[i] = 0
            else:
                self.joint_reached[i] = True
                self.apply_joint_velocities[i] = 0
            
            self.prev_speed[i] = self.apply_joint_velocities[i]
            self.prev_PID_time = time.time()
        #self.lpfw_dr_f +=1
    

        message = Float64MultiArray()
        message.data = [float(v/100) for v in self.current_calc_speed]  # Convert velocities to float
        self.vel_error_d.publish(message)

        message = Float64MultiArray()
        message.data = [float(v/100) for v in self.current_calc_speed_d_speed]  # Convert velocities to float
        self.vel_calc_speed.publish(message)

        message = Float64MultiArray()
        message.data = [float(v/100) for v in self.current_calc_speed_dm_speed]  # Convert velocities to float
        self.vel_motor_speed.publish(message)



        # Ensure that apply_joint_velocities are floats before publishing
        message = Float64MultiArray()
        message.data = [float(v/100) for v in self.apply_joint_velocities]  # Convert velocities to float
        self.velocities_publisher_.publish(message)

        # Publish reference joint states (target positions)
        reference_joint_states = Float64MultiArray()
        reference_joint_states.data = [float(v) for v in self.target_positions]  # Convert positions to float
        self.reference_value_publisher_.publish(reference_joint_states)

        # Publish current joint position and velocity (both positions and velocities need to be floats)
        current_state_message = Float64MultiArray()
        current_state_message.data = [float(p) for p in self.current_positions] + [float(v) for v in self.current_speeds]  # Convert both positions and velocities
        self.current_state_publisher_.publish(current_state_message)

        scaled_joint_velocities = [v / 100 for v in self.apply_joint_velocities]
        scaled_calc_speed = [v / 100 for v in self.current_calc_speed]

        D_error = [d * (-s) for d, s in zip(self.derivative_gain, self.prev_speed)]
        
        msg = Float64MultiArray()  # Create the Float64MultiArray message
        msg.data = D_error  # Assign the D_error list to the `data` field
        
        self.D_error_publisher_.publish(msg)  # Publish the message

        Derivative_error_C = [d * (s * 57.29) for d, s in zip(self.derivative_gain, error_derivative)]
        
        msg = Float64MultiArray()  # Create the Float64MultiArray message
        msg.data = Derivative_error_C  # Assign the D_error list to the `data` field
        
        self.Derivative_error_publisher_.publish(msg)  # Publish the message

        Derivative_error_M = [d * (-s) for d, s in zip(self.derivative_gain, self.prev_motor_speeds)]
        
        msg = Float64MultiArray()  # Create the Float64MultiArray message
        msg.data = Derivative_error_M  # Assign the D_error list to the `data` field
        
        self.Derivativem_error_publisher_.publish(msg)  # Publish the message

        # Log the scaled values
        self.get_logger().info(f"Publishing Joint Velocities (v1-v7): {scaled_joint_velocities}")
        self.get_logger().info(f"Publishing Original Calculated Velocities (v1-v7): {scaled_calc_speed}")
        
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {current_state_message.data}")
        self.get_logger().info(f"Publishing Integral Error: {self.integral_error_r}")
        self.get_logger().info(f"Publishing D Error: {[d * (-s) for d, s in zip(self.derivative_gain, self.prev_speed)]}")
        self.get_logger().info(f"Publishing Derivative Error: {[d * (s * 57.29) for d, s in zip(self.derivative_gain, error_derivative)]}")
        self.get_logger().info(f"Joint Reached Status: {self.joint_reached}")
        self.get_logger().info(f"Error: {error}")



    def update_current_positions(self):
        max_retries = 3  # Maximum number of retries
        retries = 0  # Retry counter

        while retries < max_retries:
            # Send the command to request positions (no speeds)
            s_time = time.time()
            self.serial_port.write(b"rmpv\n")  # Request motor positions
            self.get_logger().info("Requesting current motor positions...")

            response = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Updating current motor positions: {response}")

            # Check if the response is valid (only positions)
            if response.startswith("<P") and response.endswith(">"):
                parsed_data = self.parse_arduino_msg(response)  # Parse positions (no speeds)

                # Check for None values in positions, indicating errors
                if any(p is None for p in parsed_data["positions"]):
                    self.get_logger().warn("Error reading motor positions, retrying...")
                    retries += 1  # Increment retry counter
                    time.sleep(0.001)  # Optional delay before retrying
                    continue  # Retry reading the positions

                # If valid data is found, update positions
                f_time = time.time()
                m_alpha = 1 - ((1 - self.lpfw) * (self.lpfw_dr ** self.lpfw_dr_f))  # Apply filtering

                for i in range(self.num_joints):
                    self.current_positions[i] = (
                        m_alpha * parsed_data["positions"][i] + (1 - m_alpha) * self.current_positions[i]
                    )

                self.get_logger().info(f"Passed time in reading positions: {f_time - s_time:.6f} seconds")
                self.get_logger().info("Successfully read current motor positions.")
                return  # Exit the method on success

            # If response is invalid, increment retries and log a warning
            self.get_logger().warn("Invalid response, retrying...")
            retries += 1
            time.sleep(0.01)  # Optional delay before retrying

        # If retries are exhausted, stop all motors
        self.get_logger().error("Failed to read motor positions after maximum retries. Stopping all motors.")
        for motor_index in range(1, len(self.target_positions) + 1):  # Assuming motors are indexed 1 to n
            stop_command = f"stop{motor_index}\n"
            self.serial_port.write(stop_command.encode())
            self.get_logger().info(f"Sent stop command to motor {motor_index}")




    def timer_callback(self):
        if not self.poweroff:
            
            # ********************************************************************************
            joint_displacements = [current - initial for current, initial in zip(self.current_positions, self.initial_positions)]
            self.get_logger().info(f"Joint Displacements {joint_displacements}")
            self.get_logger().info(f"Current Positions {self.current_positions}")
            self.get_logger().info(f"Initial Positions {self.initial_positions}")
            # Create and populate the message
            joint_displacement_msg = Float64MultiArray()
            joint_displacement_msg.data = [
                float(current - initial)
                for current, initial in zip(self.current_positions, self.initial_positions)
            ]

            # Publish the message
            self.rel_value_publisher_.publish(joint_displacement_msg)

            # Create and populate the message with zeros
            joint_org_msg = Float64MultiArray()
            joint_org_msg.data = [0.0] * self.num_joints  # Assuming self.num_joints defines the number of joints

            # Publish the message
            self.org_value_publisher_.publish(joint_org_msg)

            self.get_logger().info(f"Joint displacements: {joint_displacements}")
            # ********************************************************************************

            self.get_logger().info(f"Done reading the current motors' state")
            t3 = time.time()
            self.calculate_pid_speeds()
            t4 = time.time()    
            self.get_logger().info(f"Time spent calculating PID speeds: {t4 - t3:.6f} seconds")
            self.get_logger().info(f"Done calculating the required motors' speeds")
            t4 = time.time()
            self.write_motor_positions_with_pid_speeds()
            t5 = time.time()    
            self.get_logger().info(f"Time spent writing motor positions with PID speeds: {t5 - t4:.6f} seconds")
    
    def voltage_callback(self):
        self.read_motor_voltage()


    def start_control_loop(self):
        # Timer for periodic updates (e.g., 1000 Hz)
        self.timer = self.create_timer(0.001, self.timer_callback)

    def voltage_checker(self):
        # Timer for periodic updates (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.voltage_callback)

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