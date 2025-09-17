// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_msgs:msg/VelTwistPose.idl
// generated code does not contain a copyright notice

#ifndef PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__STRUCT_HPP_
#define PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'vel'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_msgs__msg__VelTwistPose __attribute__((deprecated))
#else
# define DEPRECATED__plan_msgs__msg__VelTwistPose __declspec(deprecated)
#endif

namespace plan_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelTwistPose_
{
  using Type = VelTwistPose_<ContainerAllocator>;

  explicit VelTwistPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vel(_init),
    twist(_init),
    pose(_init)
  {
    (void)_init;
  }

  explicit VelTwistPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vel(_alloc, _init),
    twist(_alloc, _init),
    pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _vel_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vel_type vel;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__vel(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_msgs::msg::VelTwistPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_msgs::msg::VelTwistPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::VelTwistPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_msgs::msg::VelTwistPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_msgs__msg__VelTwistPose
    std::shared_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_msgs__msg__VelTwistPose
    std::shared_ptr<plan_msgs::msg::VelTwistPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelTwistPose_ & other) const
  {
    if (this->vel != other.vel) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelTwistPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelTwistPose_

// alias to use template instance with default allocator
using VelTwistPose =
  plan_msgs::msg::VelTwistPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_msgs

#endif  // PLAN_MSGS__MSG__DETAIL__VEL_TWIST_POSE__STRUCT_HPP_
