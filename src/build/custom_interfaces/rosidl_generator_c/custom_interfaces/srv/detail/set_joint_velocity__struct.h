// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:srv/SetJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__STRUCT_H_
#define CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetJointVelocity in the package custom_interfaces.
typedef struct custom_interfaces__srv__SetJointVelocity_Request
{
  double vq1;
  double vq2;
  double vq3;
} custom_interfaces__srv__SetJointVelocity_Request;

// Struct for a sequence of custom_interfaces__srv__SetJointVelocity_Request.
typedef struct custom_interfaces__srv__SetJointVelocity_Request__Sequence
{
  custom_interfaces__srv__SetJointVelocity_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__SetJointVelocity_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetJointVelocity in the package custom_interfaces.
typedef struct custom_interfaces__srv__SetJointVelocity_Response
{
  uint8_t structure_needs_at_least_one_member;
} custom_interfaces__srv__SetJointVelocity_Response;

// Struct for a sequence of custom_interfaces__srv__SetJointVelocity_Response.
typedef struct custom_interfaces__srv__SetJointVelocity_Response__Sequence
{
  custom_interfaces__srv__SetJointVelocity_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__SetJointVelocity_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__SET_JOINT_VELOCITY__STRUCT_H_
