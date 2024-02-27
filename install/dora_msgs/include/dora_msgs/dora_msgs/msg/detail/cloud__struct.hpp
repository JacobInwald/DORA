// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dora_msgs:msg/Cloud.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__CLOUD__STRUCT_HPP_
#define DORA_MSGS__MSG__DETAIL__CLOUD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "dora_msgs/msg/detail/pose__struct.hpp"
// Member 'scan'
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dora_msgs__msg__Cloud __attribute__((deprecated))
#else
# define DEPRECATED__dora_msgs__msg__Cloud __declspec(deprecated)
#endif

namespace dora_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Cloud_
{
  using Type = Cloud_<ContainerAllocator>;

  explicit Cloud_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    scan(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_range = 0.0;
    }
  }

  explicit Cloud_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init),
    scan(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_range = 0.0;
    }
  }

  // field types and members
  using _position_type =
    dora_msgs::msg::Pose_<ContainerAllocator>;
  _position_type position;
  using _scan_type =
    sensor_msgs::msg::LaserScan_<ContainerAllocator>;
  _scan_type scan;
  using _max_range_type =
    double;
  _max_range_type max_range;

  // setters for named parameter idiom
  Type & set__position(
    const dora_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__scan(
    const sensor_msgs::msg::LaserScan_<ContainerAllocator> & _arg)
  {
    this->scan = _arg;
    return *this;
  }
  Type & set__max_range(
    const double & _arg)
  {
    this->max_range = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dora_msgs::msg::Cloud_<ContainerAllocator> *;
  using ConstRawPtr =
    const dora_msgs::msg::Cloud_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dora_msgs::msg::Cloud_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dora_msgs::msg::Cloud_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Cloud_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Cloud_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dora_msgs::msg::Cloud_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dora_msgs::msg::Cloud_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dora_msgs::msg::Cloud_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dora_msgs::msg::Cloud_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dora_msgs__msg__Cloud
    std::shared_ptr<dora_msgs::msg::Cloud_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dora_msgs__msg__Cloud
    std::shared_ptr<dora_msgs::msg::Cloud_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Cloud_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->scan != other.scan) {
      return false;
    }
    if (this->max_range != other.max_range) {
      return false;
    }
    return true;
  }
  bool operator!=(const Cloud_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Cloud_

// alias to use template instance with default allocator
using Cloud =
  dora_msgs::msg::Cloud_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__CLOUD__STRUCT_HPP_
