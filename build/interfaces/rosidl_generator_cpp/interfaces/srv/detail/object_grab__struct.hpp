// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:srv/ObjectGrab.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__OBJECT_GRAB__STRUCT_HPP_
#define INTERFACES__SRV__DETAIL__OBJECT_GRAB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'object'
#include "interfaces/msg/detail/object_information__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interfaces__srv__ObjectGrab_Request __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__ObjectGrab_Request __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ObjectGrab_Request_
{
  using Type = ObjectGrab_Request_<ContainerAllocator>;

  explicit ObjectGrab_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : object(_init)
  {
    (void)_init;
  }

  explicit ObjectGrab_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : object(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _object_type =
    interfaces::msg::ObjectInformation_<ContainerAllocator>;
  _object_type object;

  // setters for named parameter idiom
  Type & set__object(
    const interfaces::msg::ObjectInformation_<ContainerAllocator> & _arg)
  {
    this->object = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::ObjectGrab_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::ObjectGrab_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::ObjectGrab_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::ObjectGrab_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__ObjectGrab_Request
    std::shared_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__ObjectGrab_Request
    std::shared_ptr<interfaces::srv::ObjectGrab_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectGrab_Request_ & other) const
  {
    if (this->object != other.object) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectGrab_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectGrab_Request_

// alias to use template instance with default allocator
using ObjectGrab_Request =
  interfaces::srv::ObjectGrab_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__ObjectGrab_Response __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__ObjectGrab_Response __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ObjectGrab_Response_
{
  using Type = ObjectGrab_Response_<ContainerAllocator>;

  explicit ObjectGrab_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ObjectGrab_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    interfaces::srv::ObjectGrab_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::ObjectGrab_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::ObjectGrab_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::ObjectGrab_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__ObjectGrab_Response
    std::shared_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__ObjectGrab_Response
    std::shared_ptr<interfaces::srv::ObjectGrab_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectGrab_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectGrab_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectGrab_Response_

// alias to use template instance with default allocator
using ObjectGrab_Response =
  interfaces::srv::ObjectGrab_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces

namespace interfaces
{

namespace srv
{

struct ObjectGrab
{
  using Request = interfaces::srv::ObjectGrab_Request;
  using Response = interfaces::srv::ObjectGrab_Response;
};

}  // namespace srv

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__OBJECT_GRAB__STRUCT_HPP_
