#!/usr/bin/env python3
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
        self.joint_state_subscriber_ = self.create_subscription(JointState, '/joint_states', self.calculate_joint_efforts, 10)
        
        # Publishers for joint efforts and reference joint states
        self.efforts_publisher_ = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)
        self.reference_value_publisher_ = self.create_publisher(Float64MultiArray, '/reference_joint_states/commands', 10)
        
        # Initial values
        self.command_received_ = True  # Assume a default command to hold position at (0, 0)
        self.reference_position = [0.0, 0.0]
        self.apply_joint_efforts = [0.0, 0.0]
        self.acceptable_error = 0.0001
        self.proportional_gain = [20, 200]
        self.derivative_gain = [5, 10]
        self.integral_gain = [0, 0.01]
        self.integral_error = [0.0, 0.0]
        self.i_clamp = [1.0, 450.0]

    def recieve_reference_joint_position_from_service(self, request, response):
        self.command_received_ = True
        self.reference_position = [request.rq1, request.rq2]
        self.get_logger().info(f"Request (q1,q2): ({self.reference_position[0]}, {self.reference_position[1]})")
        return response

    def calculate_joint_efforts(self, msg):
        if self.command_received_:
            joint_position = [msg.position[0], msg.position[1]]
            joint_velocity = [msg.velocity[0], msg.velocity[1]]
            error = [joint_position[0] - self.reference_position[0], joint_position[1] - self.reference_position[1]]

            # Update integral error with clamping
            self.integral_error[0] += error[0]
            self.integral_error[0] = np.clip(self.integral_error[0], -self.i_clamp[0], self.i_clamp[0])
            self.integral_error[1] += error[1]
            self.integral_error[1] = np.clip(self.integral_error[1], -self.i_clamp[1], self.i_clamp[1])

            # Calculate efforts
            if abs(error[0]) > self.acceptable_error:
                self.apply_joint_efforts[0] = -(self.proportional_gain[0] * error[0]) \
                                              - (self.derivative_gain[0] * joint_velocity[0]) \
                                              - (self.integral_gain[0] * self.integral_error[0])
            if abs(error[1]) > self.acceptable_error:
                self.apply_joint_efforts[1] = -(self.proportional_gain[1] * error[1]) \
                                              - (self.derivative_gain[1] * joint_velocity[1]) \
                                              - (self.integral_gain[1] * self.integral_error[1])

            # Publish joint efforts
            message = Float64MultiArray()
            message.data = self.apply_joint_efforts
            self.efforts_publisher_.publish(message)

            # Publish reference joint states
            reference_joint_states = Float64MultiArray()
            reference_joint_states.data = self.reference_position
            self.reference_value_publisher_.publish(reference_joint_states)

            self.get_logger().info(f"Errors (q1,q2): ({error[0]}, {error[1]})")
            self.get_logger().info(f"Publishing Joint Efforts (u1,u2): ({self.apply_joint_efforts[0]}, {self.apply_joint_efforts[1]})")
            self.get_logger().info(f"Publishing integral error: ({self.integral_error[1]})")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateController()
    node.get_logger().info("Starting Joint Effort Control.")

    # Start external commands
    os.system("ros2 run rrbot_gazebo switch_eff")
    os.system("ros2 topic pub --once /forward_effort_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        os.system("ros2 topic pub --once /forward_effort_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class RestEffortCalculator(Node):
    def __init__(self):
        super().__init__('rest_effort_calculator')
        
        # Create subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Create publisher for joint effort commands
        self.effort_publisher = self.create_publisher(
            Float64MultiArray, '/forward_effort_controller/commands', 10)

        # Create a parameter for target position
        self.declare_parameter('target_position', [0.0, 0.0])  # Default target position [q1, q2]
        
        # Define gains for PID-like controller
        self.proportional_gain = [20.0, 200.0]
        self.derivative_gain = [5.0, 10.0]
        self.integral_gain = [0.0, 0.01]
        self.integral_error = [0.0, 0.0]
        self.i_clamp = [1.0, 450.0]
        self.acceptable_error = 0.0001

    def joint_state_callback(self, msg):
        # Get the current positions and efforts for joint2 (assuming joint2 is index 1)
        joint2_position = msg.position[1]
        joint2_velocity = msg.velocity[1]
        joint2_effort = msg.effort[1]

        # Get the target position from the parameter
        target_position = self.get_parameter('target_position').get_parameter_value().double_array_value
        target_position = np.array(target_position)  # Convert to numpy array for easy handling

        # Calculate the error
        error = [joint2_position - target_position[1]]  # For simplicity, we're just controlling joint2 here

        # Update integral error with clamping
        self.integral_error[1] += error[0]
        self.integral_error[1] = np.clip(self.integral_error[1], -self.i_clamp[1], self.i_clamp[1])

        # Calculate the effort (torque) for joint2 using PID-like control
        if abs(error[0]) > self.acceptable_error:
            apply_joint_effort = -(self.proportional_gain[1] * error[0]) \
                                 - (self.derivative_gain[1] * joint2_velocity) \
                                 - (self.integral_gain[1] * self.integral_error[1])
        else:
            apply_joint_effort = 0.0  # If within acceptable error, no effort needed

        # Log the current state and calculated effort
        self.get_logger().info(f"Target Position: {target_position[1]}")
        self.get_logger().info(f"Current Position: {joint2_position}")
        self.get_logger().info(f"Error: {error[0]}")
        self.get_logger().info(f"Calculated Effort (Torque) for Joint 2: {apply_joint_effort}")

        # Publish the calculated effort
        command = Float64MultiArray()
        command.data = [0.0, apply_joint_effort]  # Only adjusting joint2 effort
        self.effort_publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = RestEffortCalculator()

    # Set the default target position, e.g., [0, 0] for both joints
    node.set_parameters([rclpy.parameter.Parameter('target_position', rclpy.Parameter.Type.DOUBLE_ARRAY, [0.0, 0.0])])

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""


