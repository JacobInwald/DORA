// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_msgs:msg/Move.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MOVE__BUILDER_HPP_
#define DORA_MSGS__MSG__DETAIL__MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_msgs/msg/detail/move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_msgs
{

namespace msg
{

namespace builder
{

class Init_Move_arg_1
{
public:
  explicit Init_Move_arg_1(::dora_msgs::msg::Move & msg)
  : msg_(msg)
  {}
  ::dora_msgs::msg::Move arg_1(::dora_msgs::msg::Move::_arg_1_type arg)
  {
    msg_.arg_1 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_msgs::msg::Move msg_;
};

class Init_Move_type
{
public:
  Init_Move_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_arg_1 type(::dora_msgs::msg::Move::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Move_arg_1(msg_);
  }

private:
  ::dora_msgs::msg::Move msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_msgs::msg::Move>()
{
  return dora_msgs::msg::builder::Init_Move_type();
}

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__MOVE__BUILDER_HPP_
