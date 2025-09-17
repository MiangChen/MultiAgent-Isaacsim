// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scene_msgs:msg/SceneModifications.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__STRUCT_HPP_
#define SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'modifications'
#include "scene_msgs/msg/detail/prim_transform__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__scene_msgs__msg__SceneModifications __attribute__((deprecated))
#else
# define DEPRECATED__scene_msgs__msg__SceneModifications __declspec(deprecated)
#endif

namespace scene_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SceneModifications_
{
  using Type = SceneModifications_<ContainerAllocator>;

  explicit SceneModifications_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SceneModifications_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _modifications_type =
    std::vector<scene_msgs::msg::PrimTransform_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<scene_msgs::msg::PrimTransform_<ContainerAllocator>>>;
  _modifications_type modifications;

  // setters for named parameter idiom
  Type & set__modifications(
    const std::vector<scene_msgs::msg::PrimTransform_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<scene_msgs::msg::PrimTransform_<ContainerAllocator>>> & _arg)
  {
    this->modifications = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scene_msgs::msg::SceneModifications_<ContainerAllocator> *;
  using ConstRawPtr =
    const scene_msgs::msg::SceneModifications_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scene_msgs::msg::SceneModifications_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scene_msgs::msg::SceneModifications_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scene_msgs__msg__SceneModifications
    std::shared_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scene_msgs__msg__SceneModifications
    std::shared_ptr<scene_msgs::msg::SceneModifications_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SceneModifications_ & other) const
  {
    if (this->modifications != other.modifications) {
      return false;
    }
    return true;
  }
  bool operator!=(const SceneModifications_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SceneModifications_

// alias to use template instance with default allocator
using SceneModifications =
  scene_msgs::msg::SceneModifications_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace scene_msgs

#endif  // SCENE_MSGS__MSG__DETAIL__SCENE_MODIFICATIONS__STRUCT_HPP_
