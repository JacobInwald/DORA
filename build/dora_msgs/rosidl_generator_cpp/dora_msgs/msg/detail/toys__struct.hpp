// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dora_msgs:msg/Toys.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOYS__STRUCT_HPP_
#define DORA_MSGS__MSG__DETAIL__TOYS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'toys'
#include "dora_msgs/msg/detail/toy__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dora_msgs__msg__Toys __attribute__((deprecated))
#else
# define DEPRECATED__dora_msgs__msg__Toys __declspec(deprecated)
#endif

namespace dora_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Toys_
{
  using Type = Toys_<ContainerAllocator>;

  explicit Toys_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Toys_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _toys_type =
    std::vector<dora_msgs::msg::Toy_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dora_msgs::msg::Toy_<ContainerAllocator>>>;
  _toys_type toys;

  // setters for named parameter idiom
  Type & set__toys(
    const std::vector<dora_msgs::msg::Toy_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dora_msgs::msg::Toy_<ContainerAllocator>>> & _arg)
  {
    this->toys = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dora_msgs::msg::Toys_<ContainerAllocator> *;
  using ConstRawPtr =
    const dora_msgs::msg::Toys_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dora_msgs::msg::Toys_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dora_msgs::msg::Toys_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Toys_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Toys_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Toys_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Toys_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dora_msgs::msg::Toys_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dora_msgs::msg::Toys_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dora_msgs__msg__Toys
    std::shared_ptr<dora_msgs::msg::Toys_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dora_msgs__msg__Toys
    std::shared_ptr<dora_msgs::msg::Toys_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Toys_ & other) const
  {
    if (this->toys != other.toys) {
      return false;
    }
    return true;
  }
  bool operator!=(const Toys_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Toys_

// alias to use template instance with default allocator
using Toys =
  dora_msgs::msg::Toys_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__TOYS__STRUCT_HPP_
