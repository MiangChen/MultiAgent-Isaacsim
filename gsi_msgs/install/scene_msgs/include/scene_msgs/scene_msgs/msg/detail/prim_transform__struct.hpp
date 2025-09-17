// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scene_msgs:msg/PrimTransform.idl
// generated code does not contain a copyright notice

#ifndef SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__STRUCT_HPP_
#define SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'transform'
#include "geometry_msgs/msg/detail/transform__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__scene_msgs__msg__PrimTransform __attribute__((deprecated))
#else
# define DEPRECATED__scene_msgs__msg__PrimTransform __declspec(deprecated)
#endif

namespace scene_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PrimTransform_
{
  using Type = PrimTransform_<ContainerAllocator>;

  explicit PrimTransform_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : transform(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->change_type = 0;
      this->prim_path = "";
    }
  }

  explicit PrimTransform_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : prim_path(_alloc),
    transform(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->change_type = 0;
      this->prim_path = "";
    }
  }

  // field types and members
  using _change_type_type =
    uint8_t;
  _change_type_type change_type;
  using _prim_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _prim_path_type prim_path;
  using _transform_type =
    geometry_msgs::msg::Transform_<ContainerAllocator>;
  _transform_type transform;

  // setters for named parameter idiom
  Type & set__change_type(
    const uint8_t & _arg)
  {
    this->change_type = _arg;
    return *this;
  }
  Type & set__prim_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->prim_path = _arg;
    return *this;
  }
  Type & set__transform(
    const geometry_msgs::msg::Transform_<ContainerAllocator> & _arg)
  {
    this->transform = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ADD =
    0u;
  // guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
  static constexpr uint8_t DELETE =
    1u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif
  static constexpr uint8_t TRANSFORM_CHANGED =
    2u;

  // pointer types
  using RawPtr =
    scene_msgs::msg::PrimTransform_<ContainerAllocator> *;
  using ConstRawPtr =
    const scene_msgs::msg::PrimTransform_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scene_msgs::msg::PrimTransform_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scene_msgs::msg::PrimTransform_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scene_msgs__msg__PrimTransform
    std::shared_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scene_msgs__msg__PrimTransform
    std::shared_ptr<scene_msgs::msg::PrimTransform_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PrimTransform_ & other) const
  {
    if (this->change_type != other.change_type) {
      return false;
    }
    if (this->prim_path != other.prim_path) {
      return false;
    }
    if (this->transform != other.transform) {
      return false;
    }
    return true;
  }
  bool operator!=(const PrimTransform_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PrimTransform_

// alias to use template instance with default allocator
using PrimTransform =
  scene_msgs::msg::PrimTransform_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PrimTransform_<ContainerAllocator>::ADD;
#endif  // __cplusplus < 201703L
// guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PrimTransform_<ContainerAllocator>::DELETE;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PrimTransform_<ContainerAllocator>::TRANSFORM_CHANGED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace scene_msgs

#endif  // SCENE_MSGS__MSG__DETAIL__PRIM_TRANSFORM__STRUCT_HPP_
