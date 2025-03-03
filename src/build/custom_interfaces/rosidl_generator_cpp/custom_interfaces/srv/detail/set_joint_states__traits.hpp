// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:srv/SetJointStates.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__TRAITS_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/srv/detail/set_joint_states__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetJointStates_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: rq1
  {
    out << "rq1: ";
    rosidl_generator_traits::value_to_yaml(msg.rq1, out);
    out << ", ";
  }

  // member: rq2
  {
    out << "rq2: ";
    rosidl_generator_traits::value_to_yaml(msg.rq2, out);
    out << ", ";
  }

  // member: rq3
  {
    out << "rq3: ";
    rosidl_generator_traits::value_to_yaml(msg.rq3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetJointStates_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rq1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rq1: ";
    rosidl_generator_traits::value_to_yaml(msg.rq1, out);
    out << "\n";
  }

  // member: rq2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rq2: ";
    rosidl_generator_traits::value_to_yaml(msg.rq2, out);
    out << "\n";
  }

  // member: rq3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rq3: ";
    rosidl_generator_traits::value_to_yaml(msg.rq3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetJointStates_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::SetJointStates_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::SetJointStates_Request & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::SetJointStates_Request>()
{
  return "custom_interfaces::srv::SetJointStates_Request";
}

template<>
inline const char * name<custom_interfaces::srv::SetJointStates_Request>()
{
  return "custom_interfaces/srv/SetJointStates_Request";
}

template<>
struct has_fixed_size<custom_interfaces::srv::SetJointStates_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::srv::SetJointStates_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::srv::SetJointStates_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetJointStates_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetJointStates_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetJointStates_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::SetJointStates_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::SetJointStates_Response & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::SetJointStates_Response>()
{
  return "custom_interfaces::srv::SetJointStates_Response";
}

template<>
inline const char * name<custom_interfaces::srv::SetJointStates_Response>()
{
  return "custom_interfaces/srv/SetJointStates_Response";
}

template<>
struct has_fixed_size<custom_interfaces::srv::SetJointStates_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::srv::SetJointStates_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::srv::SetJointStates_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interfaces::srv::SetJointStates>()
{
  return "custom_interfaces::srv::SetJointStates";
}

template<>
inline const char * name<custom_interfaces::srv::SetJointStates>()
{
  return "custom_interfaces/srv/SetJointStates";
}

template<>
struct has_fixed_size<custom_interfaces::srv::SetJointStates>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interfaces::srv::SetJointStates_Request>::value &&
    has_fixed_size<custom_interfaces::srv::SetJointStates_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interfaces::srv::SetJointStates>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interfaces::srv::SetJointStates_Request>::value &&
    has_bounded_size<custom_interfaces::srv::SetJointStates_Response>::value
  >
{
};

template<>
struct is_service<custom_interfaces::srv::SetJointStates>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interfaces::srv::SetJointStates_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interfaces::srv::SetJointStates_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__TRAITS_HPP_
