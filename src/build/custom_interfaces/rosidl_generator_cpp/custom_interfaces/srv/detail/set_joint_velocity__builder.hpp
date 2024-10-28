// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/SetJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/set_joint_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetJointVelocity_Request_vq3
{
public:
  explicit Init_SetJointVelocity_Request_vq3(::custom_interfaces::srv::SetJointVelocity_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::SetJointVelocity_Request vq3(::custom_interfaces::srv::SetJointVelocity_Request::_vq3_type arg)
  {
    msg_.vq3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::SetJointVelocity_Request msg_;
};

class Init_SetJointVelocity_Request_vq2
{
public:
  explicit Init_SetJointVelocity_Request_vq2(::custom_interfaces::srv::SetJointVelocity_Request & msg)
  : msg_(msg)
  {}
  Init_SetJointVelocity_Request_vq3 vq2(::custom_interfaces::srv::SetJointVelocity_Request::_vq2_type arg)
  {
    msg_.vq2 = std::move(arg);
    return Init_SetJointVelocity_Request_vq3(msg_);
  }

private:
  ::custom_interfaces::srv::SetJointVelocity_Request msg_;
};

class Init_SetJointVelocity_Request_vq1
{
public:
  Init_SetJointVelocity_Request_vq1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetJointVelocity_Request_vq2 vq1(::custom_interfaces::srv::SetJointVelocity_Request::_vq1_type arg)
  {
    msg_.vq1 = std::move(arg);
    return Init_SetJointVelocity_Request_vq2(msg_);
  }

private:
  ::custom_interfaces::srv::SetJointVelocity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::SetJointVelocity_Request>()
{
  return custom_interfaces::srv::builder::Init_SetJointVelocity_Request_vq1();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::SetJointVelocity_Response>()
{
  return ::custom_interfaces::srv::SetJointVelocity_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__BUILDER_HPP_
