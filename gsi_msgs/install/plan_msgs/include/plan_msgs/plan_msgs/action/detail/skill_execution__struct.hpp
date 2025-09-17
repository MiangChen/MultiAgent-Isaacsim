// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:action/SkillExecution.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__STRUCT_HPP_
#define PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'skill_request'
#include "plan_msgs/msg/detail/robot_skill__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_Goal __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_Goal __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_Goal_
{
  using Type = SkillExecution_Goal_<ContainerAllocator>;

  explicit SkillExecution_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : skill_request(_init)
  {
    (void)_init;
  }

  explicit SkillExecution_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : skill_request(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _skill_request_type =
    plan_msgs::msg::RobotSkill_<ContainerAllocator>;
  _skill_request_type skill_request;

  // setters for named parameter idiom
  Type & set__skill_request(
    const plan_msgs::msg::RobotSkill_<ContainerAllocator> & _arg)
  {
    this->skill_request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_Goal
    std::shared_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_Goal
    std::shared_ptr<plan_msgs::action::SkillExecution_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_Goal_ & other) const
  {
    if (this->skill_request != other.skill_request) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_Goal_

// alias to use template instance with default allocator
using SkillExecution_Goal =
  plan_msgs::action::SkillExecution_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_Result __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_Result __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_Result_
{
  using Type = SkillExecution_Result_<ContainerAllocator>;

  explicit SkillExecution_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SkillExecution_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_Result
    std::shared_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_Result
    std::shared_ptr<plan_msgs::action::SkillExecution_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_Result_

// alias to use template instance with default allocator
using SkillExecution_Result =
  plan_msgs::action::SkillExecution_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_Feedback __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_Feedback_
{
  using Type = SkillExecution_Feedback_<ContainerAllocator>;

  explicit SkillExecution_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  explicit SkillExecution_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  // field types and members
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_Feedback
    std::shared_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_Feedback
    std::shared_ptr<plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_Feedback_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_Feedback_

// alias to use template instance with default allocator
using SkillExecution_Feedback =
  plan_msgs::action::SkillExecution_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "plan_msgs/action/detail/skill_execution__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Request __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_SendGoal_Request_
{
  using Type = SkillExecution_SendGoal_Request_<ContainerAllocator>;

  explicit SkillExecution_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit SkillExecution_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    plan_msgs::action::SkillExecution_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const plan_msgs::action::SkillExecution_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Request
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Request
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_SendGoal_Request_

// alias to use template instance with default allocator
using SkillExecution_SendGoal_Request =
  plan_msgs::action::SkillExecution_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Response __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_SendGoal_Response_
{
  using Type = SkillExecution_SendGoal_Response_<ContainerAllocator>;

  explicit SkillExecution_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit SkillExecution_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Response
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_SendGoal_Response
    std::shared_ptr<plan_msgs::action::SkillExecution_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_SendGoal_Response_

// alias to use template instance with default allocator
using SkillExecution_SendGoal_Response =
  plan_msgs::action::SkillExecution_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs

namespace plan_msgs
{

namespace action
{

struct SkillExecution_SendGoal
{
  using Request = plan_msgs::action::SkillExecution_SendGoal_Request;
  using Response = plan_msgs::action::SkillExecution_SendGoal_Response;
};

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Request __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_GetResult_Request_
{
  using Type = SkillExecution_GetResult_Request_<ContainerAllocator>;

  explicit SkillExecution_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit SkillExecution_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Request
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Request
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_GetResult_Request_

// alias to use template instance with default allocator
using SkillExecution_GetResult_Request =
  plan_msgs::action::SkillExecution_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "plan_msgs/action/detail/skill_execution__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Response __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_GetResult_Response_
{
  using Type = SkillExecution_GetResult_Response_<ContainerAllocator>;

  explicit SkillExecution_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit SkillExecution_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    plan_msgs::action::SkillExecution_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const plan_msgs::action::SkillExecution_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Response
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_GetResult_Response
    std::shared_ptr<plan_msgs::action::SkillExecution_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_GetResult_Response_

// alias to use template instance with default allocator
using SkillExecution_GetResult_Response =
  plan_msgs::action::SkillExecution_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs

namespace plan_msgs
{

namespace action
{

struct SkillExecution_GetResult
{
  using Request = plan_msgs::action::SkillExecution_GetResult_Request;
  using Response = plan_msgs::action::SkillExecution_GetResult_Response;
};

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "plan_msgs/action/detail/skill_execution__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__SkillExecution_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__SkillExecution_FeedbackMessage __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct SkillExecution_FeedbackMessage_
{
  using Type = SkillExecution_FeedbackMessage_<ContainerAllocator>;

  explicit SkillExecution_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit SkillExecution_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const plan_msgs::action::SkillExecution_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__SkillExecution_FeedbackMessage
    std::shared_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__SkillExecution_FeedbackMessage
    std::shared_ptr<plan_msgs::action::SkillExecution_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SkillExecution_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const SkillExecution_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SkillExecution_FeedbackMessage_

// alias to use template instance with default allocator
using SkillExecution_FeedbackMessage =
  plan_msgs::action::SkillExecution_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace plan_msgs
{

namespace action
{

struct SkillExecution
{
  /// The goal message defined in the action definition.
  using Goal = plan_msgs::action::SkillExecution_Goal;
  /// The result message defined in the action definition.
  using Result = plan_msgs::action::SkillExecution_Result;
  /// The feedback message defined in the action definition.
  using Feedback = plan_msgs::action::SkillExecution_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = plan_msgs::action::SkillExecution_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = plan_msgs::action::SkillExecution_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = plan_msgs::action::SkillExecution_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct SkillExecution SkillExecution;

}  // namespace action

}  // namespace plan_msgs

#endif  // PLAN_MSGS__ACTION__DETAIL__SKILL_EXECUTION__STRUCT_HPP_
