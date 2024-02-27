// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_msgs:msg/Toy.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOY__BUILDER_HPP_
#define DORA_MSGS__MSG__DETAIL__TOY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_msgs/msg/detail/toy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_msgs
{

namespace msg
{

namespace builder
{

class Init_Toy_position
{
public:
  explicit Init_Toy_position(::dora_msgs::msg::Toy & msg)
  : msg_(msg)
  {}
  ::dora_msgs::msg::Toy position(::dora_msgs::msg::Toy::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_msgs::msg::Toy msg_;
};

class Init_Toy_conf
{
public:
  explicit Init_Toy_conf(::dora_msgs::msg::Toy & msg)
  : msg_(msg)
  {}
  Init_Toy_position conf(::dora_msgs::msg::Toy::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return Init_Toy_position(msg_);
  }

private:
  ::dora_msgs::msg::Toy msg_;
};

class Init_Toy_cls
{
public:
  explicit Init_Toy_cls(::dora_msgs::msg::Toy & msg)
  : msg_(msg)
  {}
  Init_Toy_conf cls(::dora_msgs::msg::Toy::_cls_type arg)
  {
    msg_.cls = std::move(arg);
    return Init_Toy_conf(msg_);
  }

private:
  ::dora_msgs::msg::Toy msg_;
};

class Init_Toy_header
{
public:
  Init_Toy_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Toy_cls header(::dora_msgs::msg::Toy::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Toy_cls(msg_);
  }

private:
  ::dora_msgs::msg::Toy msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_msgs::msg::Toy>()
{
  return dora_msgs::msg::builder::Init_Toy_header();
}

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__TOY__BUILDER_HPP_
