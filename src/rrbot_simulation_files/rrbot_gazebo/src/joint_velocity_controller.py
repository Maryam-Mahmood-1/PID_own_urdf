#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import os


class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        # Subscriber to listen to joint angle input
        self.joint_angle_subscriber_ = self.create_subscription(
            Float64MultiArray, '/motor_angles', self.receive_joint_angles, 10
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
        self.proportional_gain = [17, 17]
        self.derivative_gain = [0.2, 0.2]
        self.integral_gain = [0.000196, 0.000196]
        self.integral_error = [0.0, 0.0]
        self.i_clamp = [450000, 450000.0]
        self.joint_1_reached = False  # To track if joint 1 has reached its target position
        self.joint_2_previous_position = 0.0

        # Publish default reference position and start control loop
        self.publish_default_reference_position()
        self.timer_ = self.create_timer(0.1, self.control_loop)

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
        """Receive joint angle input from the topic."""
        self.reference_position = msg.data  # Assuming msg.data contains [q1, q2]
        self.command_received_ = True
        self.joint_1_reached = False  # Reset flag for joint 1
        self.get_logger().info(f"Received joint angles: {self.reference_position}")

    def calculate_joint_velocities(self, msg):
        """Calculate joint velocities to move toward target positions."""
        joint_position = [msg.position[0], msg.position[1]]
        joint_velocity = [msg.velocity[0], msg.velocity[1]]
        error = [joint_position[0] - self.reference_position[0], joint_position[1] - self.reference_position[1]]

        # Update integral error with clamping
        self.integral_error[0] += error[0]
        self.integral_error[0] = np.clip(self.integral_error[0], -self.i_clamp[0], self.i_clamp[0])
        self.integral_error[1] += error[1]
        self.integral_error[1] = np.clip(self.integral_error[1], -self.i_clamp[1], self.i_clamp[1])

        # Calculate joint 1 velocity
        if abs(error[0]) > self.acceptable_error*50:
            self.apply_joint_velocities[0] = -(self.proportional_gain[0] * error[0]) \
                                             - (self.derivative_gain[0] * joint_velocity[0]) \
                                             - (self.integral_gain[0] * self.integral_error[0])
        else:
            self.apply_joint_velocities[0] = 0.0
            self.joint_1_reached = True  # Joint 1 has reached its target position

        # For joint 2, wait until joint 1 has reached its target
        if self.joint_1_reached:
            if abs(error[1]) > self.acceptable_error:
                self.apply_joint_velocities[1] = -(self.proportional_gain[1] * error[1]) \
                                                 - (self.derivative_gain[1] * joint_velocity[1]) \
                                                 - (self.integral_gain[1] * self.integral_error[1])
            else:
                self.apply_joint_velocities[1] = 0.0
        else:
            # Hold joint 2's current position
            self.apply_joint_velocities[1] = 0.0
            self.joint_2_previous_position = joint_position[1]

        # Publish joint velocities
        message = Float64MultiArray()
        message.data = self.apply_joint_velocities
        self.velocities_publisher_.publish(message)

        # Publish reference joint states
        reference_joint_states = Float64MultiArray()
        reference_joint_states.data = self.reference_position
        self.reference_value_publisher_.publish(reference_joint_states)

        # Publish current joint position and velocity
        current_state_message = Float64MultiArray()
        current_state_message.data = joint_position + joint_velocity  # Concatenate position and velocity
        self.current_state_publisher_.publish(current_state_message)

        self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
        self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {current_state_message.data}")
        self.get_logger().info(f"Joint 1 reached: {self.joint_1_reached}")


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
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''


'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import os

class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        # Subscriber to listen to joint angle input
        self.joint_angle_subscriber_ = self.create_subscription(
            Float64MultiArray, '/motor_angles', self.receive_joint_angles, 10
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
        self.timer_ = self.create_timer(0.1, self.control_loop)

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

        # Publish current joint position and velocity
        current_state_message = Float64MultiArray()
        current_state_message.data = joint_position + joint_velocity  # Concatenate position and velocity
        self.current_state_publisher_.publish(current_state_message)

        self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
        self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {current_state_message.data}")
        self.get_logger().info(f"Publishing Integral Error: ({self.integral_error[1]})")


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
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



'''#!/usr/bin/env python3
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
        self.arduino_serial = serial.Serial('/dev/ttyACM1', 115200, timeout=1)  # Update port as needed
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

        # Timer for sending data to Arduino every 100ms
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
        """Send data to the Arduino every 100ms."""
        if self.command_received_:  # Ensure commands have been received before sending
            try:
                # Format the data to send to Arduino
                serial_data = f"#{int(self.reference_position[0])} {int(self.reference_position[1])}$\n"
                self.arduino_serial.write(serial_data.encode('utf-8'))
                self.get_logger().info(f"Sent to Arduino: {serial_data.strip()}")
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
'''


'''#!/usr/bin/env python3
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
        self.arduino_serial = serial.Serial('/dev/ttyACM1', 115200, timeout=1)  # Update port as needed
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
'''

'''#!/usr/bin/env python3
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
        self.arduino_serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Update port as needed
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
        self.timer_ = self.create_timer(0.1, self.control_loop)

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

        # Publish current joint position and velocity
        current_state_message = Float64MultiArray()
        current_state_message.data = joint_position + joint_velocity  # Concatenate position and velocity

        # Publish to the Arduino and current_joint_state topic if conditions are met
        if self.command_received_ or (abs(error[0]) < 0.01 and abs(error[1]) < 0.01):
            self.current_state_publisher_.publish(current_state_message)

            # Send joint state data to Arduino
            serial_data = f"<{current_state_message.data[0]:.2f},{current_state_message.data[1]:.2f},{current_state_message.data[2]:.2f},{current_state_message.data[3]:.2f}>\n"
            self.arduino_serial.write(serial_data.encode('utf-8'))

            self.get_logger().info(f"Sent to Arduino: {serial_data.strip()}")

        self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
        self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {current_state_message.data}")
        self.get_logger().info(f"Publishing Integral Error: ({self.integral_error[1]})")

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
'''


# Pos-vel publisher, but with pos-vel publishing and arduino serial only starting when zero position is set here.
'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import os

class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

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
        self.timer_ = self.create_timer(0.1, self.control_loop)

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

        # Publish current joint position and velocity
        current_state_message = Float64MultiArray()
        current_state_message.data = joint_position + joint_velocity  # Concatenate position and velocity

        if self.command_received_ == True:
            self.current_state_publisher_.publish(current_state_message)
        else:
            if abs(error[0]) < 0.009 and abs(error[1]) < 0.009:
                self.current_state_publisher_.publish(current_state_message)

        self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
        self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {current_state_message.data}")
        self.get_logger().info(f"Publishing Integral Error: ({self.integral_error[1]})")


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
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''



#^^^ Without Arduino, with pos-vel publisher
'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import os

class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

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
        self.timer_ = self.create_timer(0.1, self.control_loop)

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

        # Publish current joint position and velocity
        current_state_message = Float64MultiArray()
        current_state_message.data = joint_position + joint_velocity  # Concatenate position and velocity
        self.current_state_publisher_.publish(current_state_message)

        self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
        self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
        self.get_logger().info(f"Publishing Current State (Position, Velocity): {current_state_message.data}")
        self.get_logger().info(f"Publishing Integral Error: ({self.integral_error[1]})")


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
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

#Without Arduino - Without pos-velocity publisher
'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import os

class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        # Subscriber to listen to joint angle input
        self.joint_angle_subscriber_ = self.create_subscription(
            Float64MultiArray, '/joint_angle_input', self.receive_joint_angles, 10
        )
        
        # Subscriber to listen to joint states from the robot
        self.joint_state_subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.calculate_joint_velocities, 10
        )
        
        # Publishers for joint velocities and reference joint states
        self.velocities_publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.reference_value_publisher_ = self.create_publisher(Float64MultiArray, '/reference_joint_states/commands', 10)
        
        # Initial values
        self.command_received_ = False
        self.reference_position = [0.0, 0.0]  # Default target position at start
        self.apply_joint_velocities = [0.0, 0.0]
        self.acceptable_error = 0.0001
        self.proportional_gain = [0.75, 2.529]
        self.derivative_gain = [0.2, 0.2]
        self.integral_gain = [0.0002, 0.0002]
        self.integral_error = [0.0, 0.0]
        self.i_clamp = [1.0, 1450.0]

        # Publish default reference position and start control loop
        self.publish_default_reference_position()
        self.timer_ = self.create_timer(0.1, self.control_loop)

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

        self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
        self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
        self.get_logger().info(f"Publishing integral error: ({self.integral_error[1]})")


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
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

# Without (0,0) as default initial position
"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import os

class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        # Subscriber to listen to joint angle input
        self.joint_angle_subscriber_ = self.create_subscription(
            Float64MultiArray, '/joint_angle_input', self.receive_joint_angles, 10
        )
        
        # Subscriber to listen to joint states from the robot
        self.joint_state_subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.calculate_joint_velocities, 10
        )
        
        # Publishers for joint velocities and reference joint states
        self.velocities_publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.reference_value_publisher_ = self.create_publisher(Float64MultiArray, '/reference_joint_states/commands', 10)
        
        # Initial values
        self.command_received_ = False
        self.reference_position = [0.0, 0.0]
        self.apply_joint_velocities = [0.0, 0.0]
        self.acceptable_error = 0.0001
        self.proportional_gain = [2.2588, 2.529]
        self.derivative_gain = [0.2, 0.2]
        self.integral_gain = [0.0002, 0.0002]
        self.integral_error = [0.0, 0.0]
        self.i_clamp = [1.0, 450.0]

    def receive_joint_angles(self, msg):
        # Receive joint angle input from the topic
        self.reference_position = msg.data  # Assuming msg.data contains [q1, q2]
        self.command_received_ = True
        self.get_logger().info(f"Received joint angles: {self.reference_position}")

    def calculate_joint_velocities(self, msg):
        if self.command_received_:
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

            self.get_logger().info(f"Errors (q1, q2): ({error[0]}, {error[1]})")
            self.get_logger().info(f"Publishing Joint Velocities (v1, v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
            self.get_logger().info(f"Publishing integral error: ({self.integral_error[1]})")


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
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""

# Using service
"""#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from custom_interfaces.srv import SetJointStates
import numpy as np
import os

class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_controller_server')
        
        # Service to receive reference joint positions
        self.service_ = self.create_service(SetJointStates, 'joint_state_controller', self.recieve_reference_joint_position_from_service)
        
        # Subscriber to listen to joint states
        self.joint_state_subscriber_ = self.create_subscription(JointState, '/joint_states', self.calculate_joint_velocities, 10)
        
        # Publishers for joint velocities and reference joint states
        self.velocities_publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.reference_value_publisher_ = self.create_publisher(Float64MultiArray, '/reference_joint_states/commands', 10)
        
        # Initial values
        self.command_received_ = True  # Assume a default command to hold position at (0, 0)
        self.reference_position = [0.0, 0.0]
        self.apply_joint_velocities = [0.0, 0.0]
        self.acceptable_error = 0.0001
        self.proportional_gain = [1.5, 1.529]
        self.derivative_gain = [0, 0.2]
        self.integral_gain = [0.01, 0.0002]
        self.integral_error = [0.0, 0.0]
        self.i_clamp = [1.0, 450.0]

    def recieve_reference_joint_position_from_service(self, request, response):
        self.command_received_ = True
        self.reference_position = [request.rq1, request.rq2]
        self.get_logger().info(f"Request (q1,q2): ({self.reference_position[0]}, {self.reference_position[1]})")
        return response

    def calculate_joint_velocities(self, msg):
        if self.command_received_:
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

            self.get_logger().info(f"Errors (q1,q2): ({error[0]}, {error[1]})")
            self.get_logger().info(f"Publishing Joint Velocities (v1,v2): ({self.apply_joint_velocities[0]}, {self.apply_joint_velocities[1]})")
            self.get_logger().info(f"Publishing integral error: ({self.integral_error[1]})")

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
        os.system("ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""