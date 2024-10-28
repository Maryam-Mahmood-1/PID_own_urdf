// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/SetJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__SetJointVelocity_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__SetJointVelocity_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetJointVelocity_Request_
{
  using Type = SetJointVelocity_Request_<ContainerAllocator>;

  explicit SetJointVelocity_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vq1 = 0.0;
      this->vq2 = 0.0;
      this->vq3 = 0.0;
    }
  }

  explicit SetJointVelocity_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vq1 = 0.0;
      this->vq2 = 0.0;
      this->vq3 = 0.0;
    }
  }

  // field types and members
  using _vq1_type =
    double;
  _vq1_type vq1;
  using _vq2_type =
    double;
  _vq2_type vq2;
  using _vq3_type =
    double;
  _vq3_type vq3;

  // setters for named parameter idiom
  Type & set__vq1(
    const double & _arg)
  {
    this->vq1 = _arg;
    return *this;
  }
  Type & set__vq2(
    const double & _arg)
  {
    this->vq2 = _arg;
    return *this;
  }
  Type & set__vq3(
    const double & _arg)
  {
    this->vq3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__SetJointVelocity_Request
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__SetJointVelocity_Request
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetJointVelocity_Request_ & other) const
  {
    if (this->vq1 != other.vq1) {
      return false;
    }
    if (this->vq2 != other.vq2) {
      return false;
    }
    if (this->vq3 != other.vq3) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetJointVelocity_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetJointVelocity_Request_

// alias to use template instance with default allocator
using SetJointVelocity_Request =
  custom_interfaces::srv::SetJointVelocity_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__SetJointVelocity_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__SetJointVelocity_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetJointVelocity_Response_
{
  using Type = SetJointVelocity_Response_<ContainerAllocator>;

  explicit SetJointVelocity_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetJointVelocity_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__SetJointVelocity_Response
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__SetJointVelocity_Response
    std::shared_ptr<custom_interfaces::srv::SetJointVelocity_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetJointVelocity_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetJointVelocity_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetJointVelocity_Response_

// alias to use template instance with default allocator
using SetJointVelocity_Response =
  custom_interfaces::srv::SetJointVelocity_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct SetJointVelocity
{
  using Request = custom_interfaces::srv::SetJointVelocity_Request;
  using Response = custom_interfaces::srv::SetJointVelocity_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__STRUCT_HPP_
