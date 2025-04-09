// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from package_with_interfaces:msg/ObjectInformation.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__TRAITS_HPP_
#define PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "package_with_interfaces/msg/detail/object_information__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace package_with_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectInformation & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: detected
  {
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectInformation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectInformation & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace package_with_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use package_with_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const package_with_interfaces::msg::ObjectInformation & msg,
  std::ostream & out, size_t indentation = 0)
{
  package_with_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use package_with_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const package_with_interfaces::msg::ObjectInformation & msg)
{
  return package_with_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<package_with_interfaces::msg::ObjectInformation>()
{
  return "package_with_interfaces::msg::ObjectInformation";
}

template<>
inline const char * name<package_with_interfaces::msg::ObjectInformation>()
{
  return "package_with_interfaces/msg/ObjectInformation";
}

template<>
struct has_fixed_size<package_with_interfaces::msg::ObjectInformation>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<package_with_interfaces::msg::ObjectInformation>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<package_with_interfaces::msg::ObjectInformation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__TRAITS_HPP_
