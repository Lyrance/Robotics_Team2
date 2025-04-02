// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from package_with_interfaces:srv/ObjectGrab.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__SRV__DETAIL__OBJECT_GRAB__BUILDER_HPP_
#define PACKAGE_WITH_INTERFACES__SRV__DETAIL__OBJECT_GRAB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "package_with_interfaces/srv/detail/object_grab__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace package_with_interfaces
{

namespace srv
{

namespace builder
{

class Init_ObjectGrab_Request_object
{
public:
  Init_ObjectGrab_Request_object()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::package_with_interfaces::srv::ObjectGrab_Request object(::package_with_interfaces::srv::ObjectGrab_Request::_object_type arg)
  {
    msg_.object = std::move(arg);
    return std::move(msg_);
  }

private:
  ::package_with_interfaces::srv::ObjectGrab_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::package_with_interfaces::srv::ObjectGrab_Request>()
{
  return package_with_interfaces::srv::builder::Init_ObjectGrab_Request_object();
}

}  // namespace package_with_interfaces


namespace package_with_interfaces
{

namespace srv
{

namespace builder
{

class Init_ObjectGrab_Response_success
{
public:
  Init_ObjectGrab_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::package_with_interfaces::srv::ObjectGrab_Response success(::package_with_interfaces::srv::ObjectGrab_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::package_with_interfaces::srv::ObjectGrab_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::package_with_interfaces::srv::ObjectGrab_Response>()
{
  return package_with_interfaces::srv::builder::Init_ObjectGrab_Response_success();
}

}  // namespace package_with_interfaces

#endif  // PACKAGE_WITH_INTERFACES__SRV__DETAIL__OBJECT_GRAB__BUILDER_HPP_
