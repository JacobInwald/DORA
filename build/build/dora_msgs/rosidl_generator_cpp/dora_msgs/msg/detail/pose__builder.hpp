// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
#define DORA_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_msgs/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_msgs
{

namespace msg
{

namespace builder
{

class Init_Pose_rot
{
public:
  explicit Init_Pose_rot(::dora_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  ::dora_msgs::msg::Pose rot(::dora_msgs::msg::Pose::_rot_type arg)
  {
    msg_.rot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_msgs::msg::Pose msg_;
};

class Init_Pose_y
{
public:
  explicit Init_Pose_y(::dora_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_rot y(::dora_msgs::msg::Pose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Pose_rot(msg_);
  }

private:
  ::dora_msgs::msg::Pose msg_;
};

class Init_Pose_x
{
public:
  Init_Pose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose_y x(::dora_msgs::msg::Pose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pose_y(msg_);
  }

private:
  ::dora_msgs::msg::Pose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_msgs::msg::Pose>()
{
  return dora_msgs::msg::builder::Init_Pose_x();
}

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
