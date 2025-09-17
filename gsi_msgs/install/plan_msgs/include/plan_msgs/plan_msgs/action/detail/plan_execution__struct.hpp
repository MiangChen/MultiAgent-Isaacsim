// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:action/PlanExecution.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__STRUCT_HPP_
#define PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'plan'
#include "plan_msgs/msg/detail/plan__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_Goal __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_Goal __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_Goal_
{
  using Type = PlanExecution_Goal_<ContainerAllocator>;

  explicit PlanExecution_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : plan(_init)
  {
    (void)_init;
  }

  explicit PlanExecution_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : plan(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _plan_type =
    plan_msgs::msg::Plan_<ContainerAllocator>;
  _plan_type plan;

  // setters for named parameter idiom
  Type & set__plan(
    const plan_msgs::msg::Plan_<ContainerAllocator> & _arg)
  {
    this->plan = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::PlanExecution_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_Goal
    std::shared_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_Goal
    std::shared_ptr<plan_msgs::action::PlanExecution_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_Goal_ & other) const
  {
    if (this->plan != other.plan) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_Goal_

// alias to use template instance with default allocator
using PlanExecution_Goal =
  plan_msgs::action::PlanExecution_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_Result __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_Result __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_Result_
{
  using Type = PlanExecution_Result_<ContainerAllocator>;

  explicit PlanExecution_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PlanExecution_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    plan_msgs::action::PlanExecution_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_Result
    std::shared_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_Result
    std::shared_ptr<plan_msgs::action::PlanExecution_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_Result_

// alias to use template instance with default allocator
using PlanExecution_Result =
  plan_msgs::action::PlanExecution_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'skill_statuses'
#include "plan_msgs/msg/detail/skill_feedback__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_Feedback __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_Feedback_
{
  using Type = PlanExecution_Feedback_<ContainerAllocator>;

  explicit PlanExecution_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_timestep = 0l;
    }
  }

  explicit PlanExecution_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_timestep = 0l;
    }
  }

  // field types and members
  using _current_timestep_type =
    int32_t;
  _current_timestep_type current_timestep;
  using _skill_statuses_type =
    std::vector<plan_msgs::msg::SkillFeedback_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::SkillFeedback_<ContainerAllocator>>>;
  _skill_statuses_type skill_statuses;

  // setters for named parameter idiom
  Type & set__current_timestep(
    const int32_t & _arg)
  {
    this->current_timestep = _arg;
    return *this;
  }
  Type & set__skill_statuses(
    const std::vector<plan_msgs::msg::SkillFeedback_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<plan_msgs::msg::SkillFeedback_<ContainerAllocator>>> & _arg)
  {
    this->skill_statuses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_Feedback
    std::shared_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_Feedback
    std::shared_ptr<plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_Feedback_ & other) const
  {
    if (this->current_timestep != other.current_timestep) {
      return false;
    }
    if (this->skill_statuses != other.skill_statuses) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_Feedback_

// alias to use template instance with default allocator
using PlanExecution_Feedback =
  plan_msgs::action::PlanExecution_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "plan_msgs/action/detail/plan_execution__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Request __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_SendGoal_Request_
{
  using Type = PlanExecution_SendGoal_Request_<ContainerAllocator>;

  explicit PlanExecution_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PlanExecution_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    plan_msgs::action::PlanExecution_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const plan_msgs::action::PlanExecution_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Request
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Request
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_SendGoal_Request_

// alias to use template instance with default allocator
using PlanExecution_SendGoal_Request =
  plan_msgs::action::PlanExecution_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Response __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_SendGoal_Response_
{
  using Type = PlanExecution_SendGoal_Response_<ContainerAllocator>;

  explicit PlanExecution_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PlanExecution_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Response
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_SendGoal_Response
    std::shared_ptr<plan_msgs::action::PlanExecution_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_SendGoal_Response_

// alias to use template instance with default allocator
using PlanExecution_SendGoal_Response =
  plan_msgs::action::PlanExecution_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs

namespace plan_msgs
{

namespace action
{

struct PlanExecution_SendGoal
{
  using Request = plan_msgs::action::PlanExecution_SendGoal_Request;
  using Response = plan_msgs::action::PlanExecution_SendGoal_Response;
};

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Request __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_GetResult_Request_
{
  using Type = PlanExecution_GetResult_Request_<ContainerAllocator>;

  explicit PlanExecution_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PlanExecution_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Request
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Request
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_GetResult_Request_

// alias to use template instance with default allocator
using PlanExecution_GetResult_Request =
  plan_msgs::action::PlanExecution_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "plan_msgs/action/detail/plan_execution__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Response __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_GetResult_Response_
{
  using Type = PlanExecution_GetResult_Response_<ContainerAllocator>;

  explicit PlanExecution_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PlanExecution_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    plan_msgs::action::PlanExecution_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const plan_msgs::action::PlanExecution_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Response
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_GetResult_Response
    std::shared_ptr<plan_msgs::action::PlanExecution_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_GetResult_Response_

// alias to use template instance with default allocator
using PlanExecution_GetResult_Response =
  plan_msgs::action::PlanExecution_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace plan_msgs

namespace plan_msgs
{

namespace action
{

struct PlanExecution_GetResult
{
  using Request = plan_msgs::action::PlanExecution_GetResult_Request;
  using Response = plan_msgs::action::PlanExecution_GetResult_Response;
};

}  // namespace action

}  // namespace plan_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "plan_msgs/action/detail/plan_execution__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__action__PlanExecution_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__action__PlanExecution_FeedbackMessage __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PlanExecution_FeedbackMessage_
{
  using Type = PlanExecution_FeedbackMessage_<ContainerAllocator>;

  explicit PlanExecution_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PlanExecution_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const plan_msgs::action::PlanExecution_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__action__PlanExecution_FeedbackMessage
    std::shared_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__action__PlanExecution_FeedbackMessage
    std::shared_ptr<plan_msgs::action::PlanExecution_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanExecution_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanExecution_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanExecution_FeedbackMessage_

// alias to use template instance with default allocator
using PlanExecution_FeedbackMessage =
  plan_msgs::action::PlanExecution_FeedbackMessage_<std::allocator<void>>;

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

struct PlanExecution
{
  /// The goal message defined in the action definition.
  using Goal = plan_msgs::action::PlanExecution_Goal;
  /// The result message defined in the action definition.
  using Result = plan_msgs::action::PlanExecution_Result;
  /// The feedback message defined in the action definition.
  using Feedback = plan_msgs::action::PlanExecution_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = plan_msgs::action::PlanExecution_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = plan_msgs::action::PlanExecution_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = plan_msgs::action::PlanExecution_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PlanExecution PlanExecution;

}  // namespace action

}  // namespace plan_msgs

#endif  // PLAN_MSGS__ACTION__DETAIL__PLAN_EXECUTION__STRUCT_HPP_
