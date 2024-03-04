// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dora_msgs:msg/Toy.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOY__STRUCT_HPP_
#define DORA_MSGS__MSG__DETAIL__TOY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'position'
#include "dora_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dora_msgs__msg__Toy __attribute__((deprecated))
#else
# define DEPRECATED__dora_msgs__msg__Toy __declspec(deprecated)
#endif

namespace dora_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Toy_
{
  using Type = Toy_<ContainerAllocator>;

  explicit Toy_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cls = 0;
      this->conf = 0.0;
    }
  }

  explicit Toy_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cls = 0;
      this->conf = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _cls_type =
    uint8_t;
  _cls_type cls;
  using _conf_type =
    double;
  _conf_type conf;
  using _position_type =
    dora_msgs::msg::Pose_<ContainerAllocator>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cls(
    const uint8_t & _arg)
  {
    this->cls = _arg;
    return *this;
  }
  Type & set__conf(
    const double & _arg)
  {
    this->conf = _arg;
    return *this;
  }
  Type & set__position(
    const dora_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dora_msgs::msg::Toy_<ContainerAllocator> *;
  using ConstRawPtr =
    const dora_msgs::msg::Toy_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dora_msgs::msg::Toy_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dora_msgs::msg::Toy_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Toy_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Toy_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Toy_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Toy_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dora_msgs::msg::Toy_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dora_msgs::msg::Toy_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dora_msgs__msg__Toy
    std::shared_ptr<dora_msgs::msg::Toy_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dora_msgs__msg__Toy
    std::shared_ptr<dora_msgs::msg::Toy_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Toy_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->cls != other.cls) {
      return false;
    }
    if (this->conf != other.conf) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const Toy_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Toy_

// alias to use template instance with default allocator
using Toy =
  dora_msgs::msg::Toy_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__TOY__STRUCT_HPP_
