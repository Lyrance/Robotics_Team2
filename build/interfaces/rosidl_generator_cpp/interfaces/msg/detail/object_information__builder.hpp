// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/ObjectInformation.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/object_information__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectInformation_detected
{
public:
  explicit Init_ObjectInformation_detected(::interfaces::msg::ObjectInformation & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::ObjectInformation detected(::interfaces::msg::ObjectInformation::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::ObjectInformation msg_;
};

class Init_ObjectInformation_z
{
public:
  explicit Init_ObjectInformation_z(::interfaces::msg::ObjectInformation & msg)
  : msg_(msg)
  {}
  Init_ObjectInformation_detected z(::interfaces::msg::ObjectInformation::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ObjectInformation_detected(msg_);
  }

private:
  ::interfaces::msg::ObjectInformation msg_;
};

class Init_ObjectInformation_y
{
public:
  explicit Init_ObjectInformation_y(::interfaces::msg::ObjectInformation & msg)
  : msg_(msg)
  {}
  Init_ObjectInformation_z y(::interfaces::msg::ObjectInformation::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ObjectInformation_z(msg_);
  }

private:
  ::interfaces::msg::ObjectInformation msg_;
};

class Init_ObjectInformation_x
{
public:
  Init_ObjectInformation_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectInformation_y x(::interfaces::msg::ObjectInformation::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ObjectInformation_y(msg_);
  }

private:
  ::interfaces::msg::ObjectInformation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::ObjectInformation>()
{
  return interfaces::msg::builder::Init_ObjectInformation_x();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__OBJECT_INFORMATION__BUILDER_HPP_
