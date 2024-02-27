// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_msgs:msg/Toys.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOYS__BUILDER_HPP_
#define DORA_MSGS__MSG__DETAIL__TOYS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_msgs/msg/detail/toys__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_msgs
{

namespace msg
{

namespace builder
{

class Init_Toys_toys
{
public:
  Init_Toys_toys()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_msgs::msg::Toys toys(::dora_msgs::msg::Toys::_toys_type arg)
  {
    msg_.toys = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_msgs::msg::Toys msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_msgs::msg::Toys>()
{
  return dora_msgs::msg::builder::Init_Toys_toys();
}

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__TOYS__BUILDER_HPP_
