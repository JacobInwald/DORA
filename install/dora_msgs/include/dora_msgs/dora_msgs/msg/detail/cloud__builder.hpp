// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_msgs:msg/Cloud.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__CLOUD__BUILDER_HPP_
#define DORA_MSGS__MSG__DETAIL__CLOUD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_msgs/msg/detail/cloud__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_msgs
{

namespace msg
{

namespace builder
{

class Init_Cloud_max_range
{
public:
  explicit Init_Cloud_max_range(::dora_msgs::msg::Cloud & msg)
  : msg_(msg)
  {}
  ::dora_msgs::msg::Cloud max_range(::dora_msgs::msg::Cloud::_max_range_type arg)
  {
    msg_.max_range = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_msgs::msg::Cloud msg_;
};

class Init_Cloud_scan
{
public:
  explicit Init_Cloud_scan(::dora_msgs::msg::Cloud & msg)
  : msg_(msg)
  {}
  Init_Cloud_max_range scan(::dora_msgs::msg::Cloud::_scan_type arg)
  {
    msg_.scan = std::move(arg);
    return Init_Cloud_max_range(msg_);
  }

private:
  ::dora_msgs::msg::Cloud msg_;
};

class Init_Cloud_position
{
public:
  Init_Cloud_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cloud_scan position(::dora_msgs::msg::Cloud::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Cloud_scan(msg_);
  }

private:
  ::dora_msgs::msg::Cloud msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_msgs::msg::Cloud>()
{
  return dora_msgs::msg::builder::Init_Cloud_position();
}

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__CLOUD__BUILDER_HPP_
