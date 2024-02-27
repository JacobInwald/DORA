// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dora_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MAP__STRUCT_HPP_
#define DORA_MSGS__MSG__DETAIL__MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'offset'
#include "dora_msgs/msg/detail/pose__struct.hpp"
// Member 'clouds'
#include "dora_msgs/msg/detail/cloud__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dora_msgs__msg__Map __attribute__((deprecated))
#else
# define DEPRECATED__dora_msgs__msg__Map __declspec(deprecated)
#endif

namespace dora_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Map_
{
  using Type = Map_<ContainerAllocator>;

  explicit Map_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : offset(_init)
  {
    (void)_init;
  }

  explicit Map_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : offset(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _offset_type =
    dora_msgs::msg::Pose_<ContainerAllocator>;
  _offset_type offset;
  using _clouds_type =
    std::vector<dora_msgs::msg::Cloud_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dora_msgs::msg::Cloud_<ContainerAllocator>>>;
  _clouds_type clouds;

  // setters for named parameter idiom
  Type & set__offset(
    const dora_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->offset = _arg;
    return *this;
  }
  Type & set__clouds(
    const std::vector<dora_msgs::msg::Cloud_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dora_msgs::msg::Cloud_<ContainerAllocator>>> & _arg)
  {
    this->clouds = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dora_msgs::msg::Map_<ContainerAllocator> *;
  using ConstRawPtr =
    const dora_msgs::msg::Map_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dora_msgs::msg::Map_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dora_msgs::msg::Map_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Map_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Map_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Map_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Map_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dora_msgs::msg::Map_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dora_msgs::msg::Map_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dora_msgs__msg__Map
    std::shared_ptr<dora_msgs::msg::Map_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dora_msgs__msg__Map
    std::shared_ptr<dora_msgs::msg::Map_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Map_ & other) const
  {
    if (this->offset != other.offset) {
      return false;
    }
    if (this->clouds != other.clouds) {
      return false;
    }
    return true;
  }
  bool operator!=(const Map_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Map_

// alias to use template instance with default allocator
using Map =
  dora_msgs::msg::Map_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__MAP__STRUCT_HPP_
