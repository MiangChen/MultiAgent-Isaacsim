// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:msg/Plan.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__PLAN__STRUCT_HPP_
#define PLAN_MSGS__MSG__DETAIL__PLAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'steps'
#include "plan_msgs/msg/detail/timestep_skills__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__msg__Plan __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__msg__Plan __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Plan_
{
  using Type = Plan_<ContainerAllocator>;

  explicit Plan_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Plan_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _steps_type =
    std::vector<plan_msgs::msg::TimestepSkills_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::TimestepSkills_<ContainerAllocator>>>;
  _steps_type steps;

  // setters for named parameter idiom
  Type & set__steps(
    const std::vector<plan_msgs::msg::TimestepSkills_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::TimestepSkills_<ContainerAllocator>>> & _arg)
  {
    this->steps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::msg::Plan_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::msg::Plan_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::msg::Plan_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::msg::Plan_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::Plan_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::Plan_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::Plan_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::Plan_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::msg::Plan_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::msg::Plan_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__msg__Plan
    std::shared_ptr<plan_msgs::msg::Plan_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__msg__Plan
    std::shared_ptr<plan_msgs::msg::Plan_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Plan_ & other) const
  {
    if (this->steps != other.steps) {
      return false;
    }
    return true;
  }
  bool operator!=(const Plan_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Plan_

// alias to use template instance with default allocator
using Plan =
  plan_msgs::msg::Plan_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__PLAN__STRUCT_HPP_
