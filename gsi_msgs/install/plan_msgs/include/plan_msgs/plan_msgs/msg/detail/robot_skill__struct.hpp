// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:msg/RobotSkill.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__STRUCT_HPP_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'skill_list'
#include "plan_msgs/msg/detail/skill_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__msg__RobotSkill __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__msg__RobotSkill __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotSkill_
{
  using Type = RobotSkill_<ContainerAllocator>;

  explicit RobotSkill_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  explicit RobotSkill_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  // field types and members
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _skill_list_type =
    std::vector<plan_msgs::msg::SkillInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::SkillInfo_<ContainerAllocator>>>;
  _skill_list_type skill_list;

  // setters for named parameter idiom
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__skill_list(
    const std::vector<plan_msgs::msg::SkillInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::SkillInfo_<ContainerAllocator>>> & _arg)
  {
    this->skill_list = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::msg::RobotSkill_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::msg::RobotSkill_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::RobotSkill_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::RobotSkill_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__msg__RobotSkill
    std::shared_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__msg__RobotSkill
    std::shared_ptr<plan_msgs::msg::RobotSkill_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotSkill_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->skill_list != other.skill_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotSkill_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotSkill_

// alias to use template instance with default allocator
using RobotSkill =
  plan_msgs::msg::RobotSkill_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_SKILL__STRUCT_HPP_
