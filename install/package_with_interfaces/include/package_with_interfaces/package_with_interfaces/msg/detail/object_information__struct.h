// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from package_with_interfaces:msg/ObjectInformation.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__STRUCT_H_
#define PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ObjectInformation in the package package_with_interfaces.
typedef struct package_with_interfaces__msg__ObjectInformation
{
  float x;
  float y;
  float z;
  bool detected;
} package_with_interfaces__msg__ObjectInformation;

// Struct for a sequence of package_with_interfaces__msg__ObjectInformation.
typedef struct package_with_interfaces__msg__ObjectInformation__Sequence
{
  package_with_interfaces__msg__ObjectInformation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} package_with_interfaces__msg__ObjectInformation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__STRUCT_H_
