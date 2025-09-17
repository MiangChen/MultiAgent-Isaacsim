// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:msg/SkillFeedback.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__STRUCT_HPP_
#define PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__plan_msgs__msg__SkillFeedback __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__msg__SkillFeedback __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SkillFeedback_
{
  using Type = SkillFeedback_<ContainerAllocator>;

  explicit SkillFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->skill_name = "";
      this->skill_id = "";
      this->status = "";
    }
  }

  explicit SkillFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc),
    skill_name(_alloc),
    skill_id(_alloc),
    status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->skill_name = "";
      this->skill_id = "";
      this->status = "";
    }
  }

  // field types and members
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _skill_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _skill_name_type skill_name;
  using _skill_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _skill_id_type skill_id;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__skill_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->skill_name = _arg;
    return *this;
  }
  Type & set__skill_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->skill_id = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::msg::SkillFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::msg::SkillFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::SkillFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::SkillFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__msg__SkillFeedback
    std::shared_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__msg__SkillFeedback
    std::shared_ptr<plan_msgs::msg::SkillFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillFeedback_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->skill_name != other.skill_name) {
      return false;
    }
    if (this->skill_id != other.skill_id) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillFeedback_

// alias to use template instance with default allocator
using SkillFeedback =
  plan_msgs::msg::SkillFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_FEEDBACK__STRUCT_HPP_
