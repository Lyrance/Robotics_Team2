// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from package_with_interfaces:msg/ObjectInformation.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_
#define PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "package_with_interfaces/msg/detail/object_information__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace package_with_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectInformation_detected
{
public:
  explicit Init_ObjectInformation_detected(::package_with_interfaces::msg::ObjectInformation & msg)
  : msg_(msg)
  {}
  ::package_with_interfaces::msg::ObjectInformation detected(::package_with_interfaces::msg::ObjectInformation::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return std::move(msg_);
  }

private:
  ::package_with_interfaces::msg::ObjectInformation msg_;
};

class Init_ObjectInformation_z
{
public:
  explicit Init_ObjectInformation_z(::package_with_interfaces::msg::ObjectInformation & msg)
  : msg_(msg)
  {}
  Init_ObjectInformation_detected z(::package_with_interfaces::msg::ObjectInformation::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ObjectInformation_detected(msg_);
  }

private:
  ::package_with_interfaces::msg::ObjectInformation msg_;
};

class Init_ObjectInformation_y
{
public:
  explicit Init_ObjectInformation_y(::package_with_interfaces::msg::ObjectInformation & msg)
  : msg_(msg)
  {}
  Init_ObjectInformation_z y(::package_with_interfaces::msg::ObjectInformation::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ObjectInformation_z(msg_);
  }

private:
  ::package_with_interfaces::msg::ObjectInformation msg_;
};

class Init_ObjectInformation_x
{
public:
  Init_ObjectInformation_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectInformation_y x(::package_with_interfaces::msg::ObjectInformation::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ObjectInformation_y(msg_);
  }

private:
  ::package_with_interfaces::msg::ObjectInformation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::package_with_interfaces::msg::ObjectInformation>()
{
  return package_with_interfaces::msg::builder::Init_ObjectInformation_x();
}

}  // namespace package_with_interfaces

#endif  // PACKAGE_WITH_INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_
