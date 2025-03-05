// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/ObjectGrab.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__OBJECT_GRAB__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__OBJECT_GRAB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/object_grab__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
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
  ::interfaces::srv::ObjectGrab_Request object(::interfaces::srv::ObjectGrab_Request::_object_type arg)
  {
    msg_.object = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::ObjectGrab_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::ObjectGrab_Request>()
{
  return interfaces::srv::builder::Init_ObjectGrab_Request_object();
}

}  // namespace interfaces


namespace interfaces
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
  ::interfaces::srv::ObjectGrab_Response success(::interfaces::srv::ObjectGrab_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::ObjectGrab_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::ObjectGrab_Response>()
{
  return interfaces::srv::builder::Init_ObjectGrab_Response_success();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__OBJECT_GRAB__BUILDER_HPP_
