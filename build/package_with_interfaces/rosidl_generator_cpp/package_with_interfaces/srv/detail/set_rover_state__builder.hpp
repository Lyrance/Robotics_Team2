// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from package_with_interfaces:srv/SetRoverState.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__BUILDER_HPP_
#define PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "package_with_interfaces/srv/detail/set_rover_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace package_with_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetRoverState_Request_new_state
{
public:
  Init_SetRoverState_Request_new_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::package_with_interfaces::srv::SetRoverState_Request new_state(::package_with_interfaces::srv::SetRoverState_Request::_new_state_type arg)
  {
    msg_.new_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::package_with_interfaces::srv::SetRoverState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::package_with_interfaces::srv::SetRoverState_Request>()
{
  return package_with_interfaces::srv::builder::Init_SetRoverState_Request_new_state();
}

}  // namespace package_with_interfaces


namespace package_with_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetRoverState_Response_success
{
public:
  Init_SetRoverState_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::package_with_interfaces::srv::SetRoverState_Response success(::package_with_interfaces::srv::SetRoverState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::package_with_interfaces::srv::SetRoverState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::package_with_interfaces::srv::SetRoverState_Response>()
{
  return package_with_interfaces::srv::builder::Init_SetRoverState_Response_success();
}

}  // namespace package_with_interfaces

#endif  // PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__BUILDER_HPP_
