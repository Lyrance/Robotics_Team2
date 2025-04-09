// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from package_with_interfaces:srv/SetRoverState.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__STRUCT_H_
#define PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'new_state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetRoverState in the package package_with_interfaces.
typedef struct package_with_interfaces__srv__SetRoverState_Request
{
  rosidl_runtime_c__String new_state;
} package_with_interfaces__srv__SetRoverState_Request;

// Struct for a sequence of package_with_interfaces__srv__SetRoverState_Request.
typedef struct package_with_interfaces__srv__SetRoverState_Request__Sequence
{
  package_with_interfaces__srv__SetRoverState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} package_with_interfaces__srv__SetRoverState_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetRoverState in the package package_with_interfaces.
typedef struct package_with_interfaces__srv__SetRoverState_Response
{
  bool success;
} package_with_interfaces__srv__SetRoverState_Response;

// Struct for a sequence of package_with_interfaces__srv__SetRoverState_Response.
typedef struct package_with_interfaces__srv__SetRoverState_Response__Sequence
{
  package_with_interfaces__srv__SetRoverState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} package_with_interfaces__srv__SetRoverState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__STRUCT_H_
