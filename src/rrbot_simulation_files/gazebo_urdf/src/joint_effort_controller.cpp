#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <math.h> /* round, floor, ceil, trunc */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "custom_interfaces/srv/set_joint_states.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* ----------------------- Robot Parameter Definition ----------------------- */
class joint_state_controller : public rclcpp::Node
{
public:
  joint_state_controller() : Node("joint_controller_server"), count_(0)
  {
    service_ = this->create_service<custom_interfaces::srv::SetJointStates>("joint_state_controller", std::bind(&joint_state_controller::recieve_reference_joint_position_from_service, this, _1));
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&joint_state_controller::calculate_joint_efforts, this, _1));
    efforts_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);
    reference_value_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/reference_joint_states/commands", 10);
  }

private:
  void recieve_reference_joint_position_from_service(const std::shared_ptr<custom_interfaces::srv::SetJointStates::Request> request)
  {
    command_received_ = true;
    reference_position = {
        request->rq1,
        request->rq2};
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request (q1,q2): ('%f','%f')", reference_position[0], reference_position[1]);
  }

  void calculate_joint_efforts(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (command_received_)
    {
      std::vector<std::double_t> joint_position = {
          msg->position[0],
          msg->position[1]};
      std::vector<std::double_t> joint_velocity = {
          msg->velocity[0],
          msg->velocity[1]};
      std::vector<std::double_t> error = {
          joint_position[0] - reference_position[0],
          joint_position[1] - reference_position[1]};

      // Accumulate integral error with symmetrical clamping
      integral_error[0] += error[0];
      integral_error[0] = std::clamp(integral_error[0], -i_clamp[0], i_clamp[0]); // Clamping integral error for joint 1 within [-i_clamp, +i_clamp]
      integral_error[1] += error[1];
      integral_error[1] = std::clamp(integral_error[1], -i_clamp[1], i_clamp[1]); // Clamping integral error for joint 2 within [-i_clamp, +i_clamp]

      apply_joint_efforts = {0, 3.4};

      // Calculate efforts for each joint
      if (abs(error[0]) > acceptable_error) // Joint 1
        apply_joint_efforts[0] = -(proportional_gain[0] * error[0]) 
                                 - (derivative_gain[0] * joint_velocity[0]) 
                                 - (integral_gain[0] * integral_error[0]);

      if (abs(error[1]) > acceptable_error) // Joint 2
        apply_joint_efforts[1] = -(proportional_gain[1] * error[1]) 
                                 - (derivative_gain[1] * joint_velocity[1]) 
                                 - (integral_gain[1] * integral_error[1]);

      std_msgs::msg::Float64MultiArray message;
      message.data.clear();
      message.data = apply_joint_efforts;

      std_msgs::msg::Float64MultiArray reference_joint_states;
      reference_joint_states.data = reference_position;
      reference_value_publisher_->publish(reference_joint_states);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n\n\nErrors (q1,q2): ('%f','%f')", error[0], error[1]);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing Joint Efforts (u1,u2): ('%f','%f')", apply_joint_efforts[0], apply_joint_efforts[1]);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing integral error: ('%f')", integral_error[1]);
      efforts_publisher_->publish(message);
    }
  }

  // Variable Definition for class
  rclcpp::Service<custom_interfaces::srv::SetJointStates>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr efforts_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr reference_value_publisher_;

  bool command_received_ = false;
  size_t count_;
  std::vector<std::double_t> reference_position;
  std::vector<std::double_t> apply_joint_efforts = {0, 0};

  std::double_t acceptable_error = 0.0001f;
  std::vector<std::double_t> proportional_gain = {20, 200};
  std::vector<std::double_t> derivative_gain = {5, 10};
  std::vector<std::double_t> integral_gain = {0, 0.01}; // Integral gains for joints 1 and 2

  std::vector<std::double_t> integral_error = {0, 0}; // To store accumulated integral error for each joint
  std::vector<std::double_t> i_clamp = {1.0, 450}; // Symmetrical clamping limits for integral term for each joint
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joint_state_controller>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Joint Effort Control.");
  system("ros2 run rrbot_gazebo switch_eff");
  system("ros2 topic pub --once /forward_effort_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'");
  rclcpp::spin(node);
  rclcpp::shutdown();
  system("ros2 topic pub --once /forward_effort_controller/commands std_msgs/msg/Float64MultiArray 'data: [0,0]'");
  return 0;
}
