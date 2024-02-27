// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dora_msgs:msg/Move.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MOVE__STRUCT_HPP_
#define DORA_MSGS__MSG__DETAIL__MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dora_msgs__msg__Move __attribute__((deprecated))
#else
# define DEPRECATED__dora_msgs__msg__Move __declspec(deprecated)
#endif

namespace dora_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Move_
{
  using Type = Move_<ContainerAllocator>;

  explicit Move_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->arg_1 = 0.0;
    }
  }

  explicit Move_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->arg_1 = 0.0;
    }
  }

  // field types and members
  using _type_type =
    int8_t;
  _type_type type;
  using _arg_1_type =
    double;
  _arg_1_type arg_1;

  // setters for named parameter idiom
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__arg_1(
    const double & _arg)
  {
    this->arg_1 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dora_msgs::msg::Move_<ContainerAllocator> *;
  using ConstRawPtr =
    const dora_msgs::msg::Move_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dora_msgs::msg::Move_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dora_msgs::msg::Move_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Move_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Move_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Move_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Move_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dora_msgs::msg::Move_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dora_msgs::msg::Move_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dora_msgs__msg__Move
    std::shared_ptr<dora_msgs::msg::Move_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dora_msgs__msg__Move
    std::shared_ptr<dora_msgs::msg::Move_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Move_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->arg_1 != other.arg_1) {
      return false;
    }
    return true;
  }
  bool operator!=(const Move_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Move_

// alias to use template instance with default allocator
using Move =
  dora_msgs::msg::Move_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__MOVE__STRUCT_HPP_
