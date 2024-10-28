// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/SetJointStates.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__SetJointStates_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__SetJointStates_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetJointStates_Request_
{
  using Type = SetJointStates_Request_<ContainerAllocator>;

  explicit SetJointStates_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rq1 = 0.0;
      this->rq2 = 0.0;
      this->rq3 = 0.0;
    }
  }

  explicit SetJointStates_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rq1 = 0.0;
      this->rq2 = 0.0;
      this->rq3 = 0.0;
    }
  }

  // field types and members
  using _rq1_type =
    double;
  _rq1_type rq1;
  using _rq2_type =
    double;
  _rq2_type rq2;
  using _rq3_type =
    double;
  _rq3_type rq3;

  // setters for named parameter idiom
  Type & set__rq1(
    const double & _arg)
  {
    this->rq1 = _arg;
    return *this;
  }
  Type & set__rq2(
    const double & _arg)
  {
    this->rq2 = _arg;
    return *this;
  }
  Type & set__rq3(
    const double & _arg)
  {
    this->rq3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__SetJointStates_Request
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__SetJointStates_Request
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetJointStates_Request_ & other) const
  {
    if (this->rq1 != other.rq1) {
      return false;
    }
    if (this->rq2 != other.rq2) {
      return false;
    }
    if (this->rq3 != other.rq3) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetJointStates_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetJointStates_Request_

// alias to use template instance with default allocator
using SetJointStates_Request =
  custom_interfaces::srv::SetJointStates_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__SetJointStates_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__SetJointStates_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetJointStates_Response_
{
  using Type = SetJointStates_Response_<ContainerAllocator>;

  explicit SetJointStates_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetJointStates_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__SetJointStates_Response
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__SetJointStates_Response
    std::shared_ptr<custom_interfaces::srv::SetJointStates_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetJointStates_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetJointStates_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetJointStates_Response_

// alias to use template instance with default allocator
using SetJointStates_Response =
  custom_interfaces::srv::SetJointStates_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct SetJointStates
{
  using Request = custom_interfaces::srv::SetJointStates_Request;
  using Response = custom_interfaces::srv::SetJointStates_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_STATES__STRUCT_HPP_
