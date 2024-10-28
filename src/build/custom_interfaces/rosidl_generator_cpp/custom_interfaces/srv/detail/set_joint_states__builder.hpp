// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/SetJointStates.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/set_joint_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetJointStates_Request_rq3
{
public:
  explicit Init_SetJointStates_Request_rq3(::custom_interfaces::srv::SetJointStates_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::SetJointStates_Request rq3(::custom_interfaces::srv::SetJointStates_Request::_rq3_type arg)
  {
    msg_.rq3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::SetJointStates_Request msg_;
};

class Init_SetJointStates_Request_rq2
{
public:
  explicit Init_SetJointStates_Request_rq2(::custom_interfaces::srv::SetJointStates_Request & msg)
  : msg_(msg)
  {}
  Init_SetJointStates_Request_rq3 rq2(::custom_interfaces::srv::SetJointStates_Request::_rq2_type arg)
  {
    msg_.rq2 = std::move(arg);
    return Init_SetJointStates_Request_rq3(msg_);
  }

private:
  ::custom_interfaces::srv::SetJointStates_Request msg_;
};

class Init_SetJointStates_Request_rq1
{
public:
  Init_SetJointStates_Request_rq1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetJointStates_Request_rq2 rq1(::custom_interfaces::srv::SetJointStates_Request::_rq1_type arg)
  {
    msg_.rq1 = std::move(arg);
    return Init_SetJointStates_Request_rq2(msg_);
  }

private:
  ::custom_interfaces::srv::SetJointStates_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::SetJointStates_Request>()
{
  return custom_interfaces::srv::builder::Init_SetJointStates_Request_rq1();
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
auto build<::custom_interfaces::srv::SetJointStates_Response>()
{
  return ::custom_interfaces::srv::SetJointStates_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__BUILDER_HPP_
