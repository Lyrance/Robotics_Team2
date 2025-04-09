// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from package_with_interfaces:srv/SetRoverState.idl
// generated code does not contain a copyright notice

#ifndef PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__STRUCT_HPP_
#define PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__package_with_interfaces__srv__SetRoverState_Request __attribute__((deprecated))
#else
# define DEPRECATED__package_with_interfaces__srv__SetRoverState_Request __declspec(deprecated)
#endif

namespace package_with_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetRoverState_Request_
{
  using Type = SetRoverState_Request_<ContainerAllocator>;

  explicit SetRoverState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_state = "";
    }
  }

  explicit SetRoverState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : new_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_state = "";
    }
  }

  // field types and members
  using _new_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _new_state_type new_state;

  // setters for named parameter idiom
  Type & set__new_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->new_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__package_with_interfaces__srv__SetRoverState_Request
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__package_with_interfaces__srv__SetRoverState_Request
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetRoverState_Request_ & other) const
  {
    if (this->new_state != other.new_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetRoverState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetRoverState_Request_

// alias to use template instance with default allocator
using SetRoverState_Request =
  package_with_interfaces::srv::SetRoverState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace package_with_interfaces


#ifndef _WIN32
# define DEPRECATED__package_with_interfaces__srv__SetRoverState_Response __attribute__((deprecated))
#else
# define DEPRECATED__package_with_interfaces__srv__SetRoverState_Response __declspec(deprecated)
#endif

namespace package_with_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetRoverState_Response_
{
  using Type = SetRoverState_Response_<ContainerAllocator>;

  explicit SetRoverState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetRoverState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__package_with_interfaces__srv__SetRoverState_Response
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__package_with_interfaces__srv__SetRoverState_Response
    std::shared_ptr<package_with_interfaces::srv::SetRoverState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetRoverState_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetRoverState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetRoverState_Response_

// alias to use template instance with default allocator
using SetRoverState_Response =
  package_with_interfaces::srv::SetRoverState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace package_with_interfaces

namespace package_with_interfaces
{

namespace srv
{

struct SetRoverState
{
  using Request = package_with_interfaces::srv::SetRoverState_Request;
  using Response = package_with_interfaces::srv::SetRoverState_Response;
};

}  // namespace srv

}  // namespace package_with_interfaces

#endif  // PACKAGE_WITH_INTERFACES__SRV__DETAIL__SET_ROVER_STATE__STRUCT_HPP_
