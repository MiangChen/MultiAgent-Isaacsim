// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from message_interface:msg/PathTracingCmd.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__STRUCT_HPP_
#define MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__message_interface__msg__PathTracingCmd __attribute__((deprecated))
#else
# define DEPRECATED__message_interface__msg__PathTracingCmd __declspec(deprecated)
#endif

namespace message_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PathTracingCmd_
{
  using Type = PathTracingCmd_<ContainerAllocator>;

  explicit PathTracingCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->velocity = 0.0;
      this->omega = 0.0;
      this->complete = false;
    }
  }

  explicit PathTracingCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->velocity = 0.0;
      this->omega = 0.0;
      this->complete = false;
    }
  }

  // field types and members
  using _index_type =
    int32_t;
  _index_type index;
  using _velocity_type =
    double;
  _velocity_type velocity;
  using _omega_type =
    double;
  _omega_type omega;
  using _complete_type =
    bool;
  _complete_type complete;

  // setters for named parameter idiom
  Type & set__index(
    const int32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__velocity(
    const double & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__omega(
    const double & _arg)
  {
    this->omega = _arg;
    return *this;
  }
  Type & set__complete(
    const bool & _arg)
  {
    this->complete = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    message_interface::msg::PathTracingCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const message_interface::msg::PathTracingCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      message_interface::msg::PathTracingCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      message_interface::msg::PathTracingCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__message_interface__msg__PathTracingCmd
    std::shared_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__message_interface__msg__PathTracingCmd
    std::shared_ptr<message_interface::msg::PathTracingCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathTracingCmd_ & other) const
  {
    if (this->index != other.index) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->omega != other.omega) {
      return false;
    }
    if (this->complete != other.complete) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathTracingCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathTracingCmd_

// alias to use template instance with default allocator
using PathTracingCmd =
  message_interface::msg::PathTracingCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace message_interface

#endif  // MESSAGE_INTERFACE__MSG__DETAIL__PATH_TRACING_CMD__STRUCT_HPP_
