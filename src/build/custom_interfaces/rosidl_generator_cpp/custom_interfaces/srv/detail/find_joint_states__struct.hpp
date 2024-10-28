// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/FindJointStates.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__FIND_JOINT_STATES__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__FIND_JOINT_STATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__FindJointStates_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__FindJointStates_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FindJointStates_Request_
{
  using Type = FindJointStates_Request_<ContainerAllocator>;

  explicit FindJointStates_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  explicit FindJointStates_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__FindJointStates_Request
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__FindJointStates_Request
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FindJointStates_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const FindJointStates_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FindJointStates_Request_

// alias to use template instance with default allocator
using FindJointStates_Request =
  custom_interfaces::srv::FindJointStates_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__FindJointStates_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__FindJointStates_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FindJointStates_Response_
{
  using Type = FindJointStates_Response_<ContainerAllocator>;

  explicit FindJointStates_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q11 = 0.0;
      this->q21 = 0.0;
      this->q31 = 0.0;
      this->q12 = 0.0;
      this->q22 = 0.0;
      this->q32 = 0.0;
    }
  }

  explicit FindJointStates_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q11 = 0.0;
      this->q21 = 0.0;
      this->q31 = 0.0;
      this->q12 = 0.0;
      this->q22 = 0.0;
      this->q32 = 0.0;
    }
  }

  // field types and members
  using _q11_type =
    double;
  _q11_type q11;
  using _q21_type =
    double;
  _q21_type q21;
  using _q31_type =
    double;
  _q31_type q31;
  using _q12_type =
    double;
  _q12_type q12;
  using _q22_type =
    double;
  _q22_type q22;
  using _q32_type =
    double;
  _q32_type q32;

  // setters for named parameter idiom
  Type & set__q11(
    const double & _arg)
  {
    this->q11 = _arg;
    return *this;
  }
  Type & set__q21(
    const double & _arg)
  {
    this->q21 = _arg;
    return *this;
  }
  Type & set__q31(
    const double & _arg)
  {
    this->q31 = _arg;
    return *this;
  }
  Type & set__q12(
    const double & _arg)
  {
    this->q12 = _arg;
    return *this;
  }
  Type & set__q22(
    const double & _arg)
  {
    this->q22 = _arg;
    return *this;
  }
  Type & set__q32(
    const double & _arg)
  {
    this->q32 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__FindJointStates_Response
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__FindJointStates_Response
    std::shared_ptr<custom_interfaces::srv::FindJointStates_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FindJointStates_Response_ & other) const
  {
    if (this->q11 != other.q11) {
      return false;
    }
    if (this->q21 != other.q21) {
      return false;
    }
    if (this->q31 != other.q31) {
      return false;
    }
    if (this->q12 != other.q12) {
      return false;
    }
    if (this->q22 != other.q22) {
      return false;
    }
    if (this->q32 != other.q32) {
      return false;
    }
    return true;
  }
  bool operator!=(const FindJointStates_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FindJointStates_Response_

// alias to use template instance with default allocator
using FindJointStates_Response =
  custom_interfaces::srv::FindJointStates_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct FindJointStates
{
  using Request = custom_interfaces::srv::FindJointStates_Request;
  using Response = custom_interfaces::srv::FindJointStates_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__FIND_JOINT_STATES__STRUCT_HPP_
