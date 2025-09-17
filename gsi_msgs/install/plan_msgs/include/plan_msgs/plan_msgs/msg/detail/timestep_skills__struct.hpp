// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:msg/TimestepSkills.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__STRUCT_HPP_
#define PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'robots'
#include "plan_msgs/msg/detail/robot_skill__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__msg__TimestepSkills __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__msg__TimestepSkills __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TimestepSkills_
{
  using Type = TimestepSkills_<ContainerAllocator>;

  explicit TimestepSkills_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestep = 0l;
    }
  }

  explicit TimestepSkills_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestep = 0l;
    }
  }

  // field types and members
  using _timestep_type =
    int32_t;
  _timestep_type timestep;
  using _robots_type =
    std::vector<plan_msgs::msg::RobotSkill_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::RobotSkill_<ContainerAllocator>>>;
  _robots_type robots;

  // setters for named parameter idiom
  Type & set__timestep(
    const int32_t & _arg)
  {
    this->timestep = _arg;
    return *this;
  }
  Type & set__robots(
    const std::vector<plan_msgs::msg::RobotSkill_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::RobotSkill_<ContainerAllocator>>> & _arg)
  {
    this->robots = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::msg::TimestepSkills_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::msg::TimestepSkills_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::TimestepSkills_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::TimestepSkills_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__msg__TimestepSkills
    std::shared_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__msg__TimestepSkills
    std::shared_ptr<plan_msgs::msg::TimestepSkills_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimestepSkills_ & other) const
  {
    if (this->timestep != other.timestep) {
      return false;
    }
    if (this->robots != other.robots) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimestepSkills_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimestepSkills_

// alias to use template instance with default allocator
using TimestepSkills =
  plan_msgs::msg::TimestepSkills_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__TIMESTEP_SKILLS__STRUCT_HPP_
