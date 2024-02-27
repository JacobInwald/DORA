// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dora_srvs:srv/LoopCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__LOOP_CMD__STRUCT_HPP_
#define DORA_SRVS__SRV__DETAIL__LOOP_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dora_srvs__srv__LoopCmd_Request __attribute__((deprecated))
#else
# define DEPRECATED__dora_srvs__srv__LoopCmd_Request __declspec(deprecated)
#endif

namespace dora_srvs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LoopCmd_Request_
{
  using Type = LoopCmd_Request_<ContainerAllocator>;

  explicit LoopCmd_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
    }
  }

  explicit LoopCmd_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
    }
  }

  // field types and members
  using _start_type =
    bool;
  _start_type start;

  // setters for named parameter idiom
  Type & set__start(
    const bool & _arg)
  {
    this->start = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dora_srvs::srv::LoopCmd_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dora_srvs::srv::LoopCmd_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dora_srvs::srv::LoopCmd_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dora_srvs::srv::LoopCmd_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dora_srvs__srv__LoopCmd_Request
    std::shared_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dora_srvs__srv__LoopCmd_Request
    std::shared_ptr<dora_srvs::srv::LoopCmd_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoopCmd_Request_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoopCmd_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoopCmd_Request_

// alias to use template instance with default allocator
using LoopCmd_Request =
  dora_srvs::srv::LoopCmd_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dora_srvs


#ifndef _WIN32
# define DEPRECATED__dora_srvs__srv__LoopCmd_Response __attribute__((deprecated))
#else
# define DEPRECATED__dora_srvs__srv__LoopCmd_Response __declspec(deprecated)
#endif

namespace dora_srvs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LoopCmd_Response_
{
  using Type = LoopCmd_Response_<ContainerAllocator>;

  explicit LoopCmd_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  explicit LoopCmd_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dora_srvs::srv::LoopCmd_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dora_srvs::srv::LoopCmd_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dora_srvs::srv::LoopCmd_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dora_srvs::srv::LoopCmd_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dora_srvs__srv__LoopCmd_Response
    std::shared_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dora_srvs__srv__LoopCmd_Response
    std::shared_ptr<dora_srvs::srv::LoopCmd_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoopCmd_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoopCmd_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoopCmd_Response_

// alias to use template instance with default allocator
using LoopCmd_Response =
  dora_srvs::srv::LoopCmd_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dora_srvs

namespace dora_srvs
{

namespace srv
{

struct LoopCmd
{
  using Request = dora_srvs::srv::LoopCmd_Request;
  using Response = dora_srvs::srv::LoopCmd_Response;
};

}  // namespace srv

}  // namespace dora_srvs

#endif  // DORA_SRVS__SRV__DETAIL__LOOP_CMD__STRUCT_HPP_
