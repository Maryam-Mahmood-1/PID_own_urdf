// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/SetEndEffectorVelocity.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__SET_END_EFFECTOR_VELOCITY__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__SET_END_EFFECTOR_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/set_end_effector_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetEndEffectorVelocity_Request_vz
{
public:
  explicit Init_SetEndEffectorVelocity_Request_vz(::custom_interfaces::srv::SetEndEffectorVelocity_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::SetEndEffectorVelocity_Request vz(::custom_interfaces::srv::SetEndEffectorVelocity_Request::_vz_type arg)
  {
    msg_.vz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::SetEndEffectorVelocity_Request msg_;
};

class Init_SetEndEffectorVelocity_Request_vy
{
public:
  explicit Init_SetEndEffectorVelocity_Request_vy(::custom_interfaces::srv::SetEndEffectorVelocity_Request & msg)
  : msg_(msg)
  {}
  Init_SetEndEffectorVelocity_Request_vz vy(::custom_interfaces::srv::SetEndEffectorVelocity_Request::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_SetEndEffectorVelocity_Request_vz(msg_);
  }

private:
  ::custom_interfaces::srv::SetEndEffectorVelocity_Request msg_;
};

class Init_SetEndEffectorVelocity_Request_vx
{
public:
  Init_SetEndEffectorVelocity_Request_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetEndEffectorVelocity_Request_vy vx(::custom_interfaces::srv::SetEndEffectorVelocity_Request::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_SetEndEffectorVelocity_Request_vy(msg_);
  }

private:
  ::custom_interfaces::srv::SetEndEffectorVelocity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::SetEndEffectorVelocity_Request>()
{
  return custom_interfaces::srv::builder::Init_SetEndEffectorVelocity_Request_vx();
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
auto build<::custom_interfaces::srv::SetEndEffectorVelocity_Response>()
{
  return ::custom_interfaces::srv::SetEndEffectorVelocity_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__SET_END_EFFECTOR_VELOCITY__BUILDER_HPP_
