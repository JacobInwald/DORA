// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MAP__BUILDER_HPP_
#define DORA_MSGS__MSG__DETAIL__MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_msgs/msg/detail/map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_msgs
{

namespace msg
{

namespace builder
{

class Init_Map_clouds
{
public:
  explicit Init_Map_clouds(::dora_msgs::msg::Map & msg)
  : msg_(msg)
  {}
  ::dora_msgs::msg::Map clouds(::dora_msgs::msg::Map::_clouds_type arg)
  {
    msg_.clouds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_msgs::msg::Map msg_;
};

class Init_Map_offset
{
public:
  Init_Map_offset()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Map_clouds offset(::dora_msgs::msg::Map::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return Init_Map_clouds(msg_);
  }

private:
  ::dora_msgs::msg::Map msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_msgs::msg::Map>()
{
  return dora_msgs::msg::builder::Init_Map_offset();
}

}  // namespace dora_msgs

#endif  // DORA_MSGS__MSG__DETAIL__MAP__BUILDER_HPP_
