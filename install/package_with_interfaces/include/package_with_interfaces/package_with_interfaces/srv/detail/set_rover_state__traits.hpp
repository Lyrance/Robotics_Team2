// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from package_with_interfaces:srv/SetRoverState.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__TRAITS_HPP_
#define PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "package_with_interfaces/srv/detail/set_rover_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace package_with_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetRoverState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: new_state
  {
    out << "new_state: ";
    rosidl_generator_traits::value_to_yaml(msg.new_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetRoverState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: new_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_state: ";
    rosidl_generator_traits::value_to_yaml(msg.new_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetRoverState_Request & msg, bool use_flow_style = false)
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

}  // namespace package_with_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use package_with_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const package_with_interfaces::srv::SetRoverState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  package_with_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use package_with_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const package_with_interfaces::srv::SetRoverState_Request & msg)
{
  return package_with_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<package_with_interfaces::srv::SetRoverState_Request>()
{
  return "package_with_interfaces::srv::SetRoverState_Request";
}

template<>
inline const char * name<package_with_interfaces::srv::SetRoverState_Request>()
{
  return "package_with_interfaces/srv/SetRoverState_Request";
}

template<>
struct has_fixed_size<package_with_interfaces::srv::SetRoverState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<package_with_interfaces::srv::SetRoverState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<package_with_interfaces::srv::SetRoverState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace package_with_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetRoverState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetRoverState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetRoverState_Response & msg, bool use_flow_style = false)
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

}  // namespace package_with_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use package_with_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const package_with_interfaces::srv::SetRoverState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  package_with_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use package_with_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const package_with_interfaces::srv::SetRoverState_Response & msg)
{
  return package_with_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<package_with_interfaces::srv::SetRoverState_Response>()
{
  return "package_with_interfaces::srv::SetRoverState_Response";
}

template<>
inline const char * name<package_with_interfaces::srv::SetRoverState_Response>()
{
  return "package_with_interfaces/srv/SetRoverState_Response";
}

template<>
struct has_fixed_size<package_with_interfaces::srv::SetRoverState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<package_with_interfaces::srv::SetRoverState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<package_with_interfaces::srv::SetRoverState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<package_with_interfaces::srv::SetRoverState>()
{
  return "package_with_interfaces::srv::SetRoverState";
}

template<>
inline const char * name<package_with_interfaces::srv::SetRoverState>()
{
  return "package_with_interfaces/srv/SetRoverState";
}

template<>
struct has_fixed_size<package_with_interfaces::srv::SetRoverState>
  : std::integral_constant<
    bool,
    has_fixed_size<package_with_interfaces::srv::SetRoverState_Request>::value &&
    has_fixed_size<package_with_interfaces::srv::SetRoverState_Response>::value
  >
{
};

template<>
struct has_bounded_size<package_with_interfaces::srv::SetRoverState>
  : std::integral_constant<
    bool,
    has_bounded_size<package_with_interfaces::srv::SetRoverState_Request>::value &&
    has_bounded_size<package_with_interfaces::srv::SetRoverState_Response>::value
  >
{
};

template<>
struct is_service<package_with_interfaces::srv::SetRoverState>
  : std::true_type
{
};

template<>
struct is_service_request<package_with_interfaces::srv::SetRoverState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<package_with_interfaces::srv::SetRoverState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__TRAITS_HPP_
