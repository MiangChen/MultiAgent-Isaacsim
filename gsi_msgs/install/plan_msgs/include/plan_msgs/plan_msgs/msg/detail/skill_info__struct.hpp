// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:msg/SkillInfo.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__SKILL_INFO__STRUCT_HPP_
#define PLAN_MSGS__MSG__DETAIL__SKILL_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'params'
#include "plan_msgs/msg/detail/parameter__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__msg__SkillInfo __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__msg__SkillInfo __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SkillInfo_
{
  using Type = SkillInfo_<ContainerAllocator>;

  explicit SkillInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->skill = "";
      this->object_id = "";
      this->task_id = "";
      this->status = 0l;
    }
  }

  explicit SkillInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : skill(_alloc),
    object_id(_alloc),
    task_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->skill = "";
      this->object_id = "";
      this->task_id = "";
      this->status = 0l;
    }
  }

  // field types and members
  using _skill_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _skill_type skill;
  using _params_type =
    std::vector<plan_msgs::msg::Parameter_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::Parameter_<ContainerAllocator>>>;
  _params_type params;
  using _object_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _object_id_type object_id;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _status_type =
    int32_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__skill(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->skill = _arg;
    return *this;
  }
  Type & set__params(
    const std::vector<plan_msgs::msg::Parameter_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::Parameter_<ContainerAllocator>>> & _arg)
  {
    this->params = _arg;
    return *this;
  }
  Type & set__object_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->object_id = _arg;
    return *this;
  }
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__status(
    const int32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::msg::SkillInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::msg::SkillInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::SkillInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::SkillInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__msg__SkillInfo
    std::shared_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__msg__SkillInfo
    std::shared_ptr<plan_msgs::msg::SkillInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillInfo_ & other) const
  {
    if (this->skill != other.skill) {
      return false;
    }
    if (this->params != other.params) {
      return false;
    }
    if (this->object_id != other.object_id) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillInfo_

// alias to use template instance with default allocator
using SkillInfo =
  plan_msgs::msg::SkillInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__SKILL_INFO__STRUCT_HPP_
