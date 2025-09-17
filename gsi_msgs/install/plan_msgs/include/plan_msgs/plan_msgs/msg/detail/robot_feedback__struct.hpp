// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:msg/RobotFeedback.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__STRUCT_HPP_
#define PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'skill_feedback'
#include "plan_msgs/msg/detail/skill_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__msg__RobotFeedback __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__msg__RobotFeedback __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotFeedback_
{
  using Type = RobotFeedback_<ContainerAllocator>;

  explicit RobotFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : skill_feedback(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  explicit RobotFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc),
    skill_feedback(_alloc, _init)
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
  using _skill_feedback_type =
    plan_msgs::msg::SkillInfo_<ContainerAllocator>;
  _skill_feedback_type skill_feedback;

  // setters for named parameter idiom
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__skill_feedback(
    const plan_msgs::msg::SkillInfo_<ContainerAllocator> & _arg)
  {
    this->skill_feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::msg::RobotFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::msg::RobotFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::RobotFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::RobotFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__msg__RobotFeedback
    std::shared_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__msg__RobotFeedback
    std::shared_ptr<plan_msgs::msg::RobotFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotFeedback_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->skill_feedback != other.skill_feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotFeedback_

// alias to use template instance with default allocator
using RobotFeedback =
  plan_msgs::msg::RobotFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__ROBOT_FEEDBACK__STRUCT_HPP_
