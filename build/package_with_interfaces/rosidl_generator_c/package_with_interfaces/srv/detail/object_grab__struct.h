// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from package_with_interfaces:srv/ObjectGrab.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__SRV__DETAIL__OBJECT_GRAB__STRUCT_H_
#define PACKAGE_WITH_INTERFACES__SRV__DETAIL__OBJECT_GRAB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'object'
#include "package_with_interfaces/msg/detail/object_information__struct.h"

/// Struct defined in srv/ObjectGrab in the package package_with_interfaces.
typedef struct package_with_interfaces__srv__ObjectGrab_Request
{
  package_with_interfaces__msg__ObjectInformation object;
} package_with_interfaces__srv__ObjectGrab_Request;

// Struct for a sequence of package_with_interfaces__srv__ObjectGrab_Request.
typedef struct package_with_interfaces__srv__ObjectGrab_Request__Sequence
{
  package_with_interfaces__srv__ObjectGrab_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} package_with_interfaces__srv__ObjectGrab_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ObjectGrab in the package package_with_interfaces.
typedef struct package_with_interfaces__srv__ObjectGrab_Response
{
  bool success;
} package_with_interfaces__srv__ObjectGrab_Response;

// Struct for a sequence of package_with_interfaces__srv__ObjectGrab_Response.
typedef struct package_with_interfaces__srv__ObjectGrab_Response__Sequence
{
  package_with_interfaces__srv__ObjectGrab_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} package_with_interfaces__srv__ObjectGrab_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PACKAGE_WITH_INTERFACES__SRV__DETAIL__OBJECT_GRAB__STRUCT_H_
