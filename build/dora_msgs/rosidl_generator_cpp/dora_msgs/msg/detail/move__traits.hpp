// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_msgs:msg/Move.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MOVE__TRAITS_HPP_
#define DORA_MSGS__MSG__DETAIL__MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_msgs/msg/detail/move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dora_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Move & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: arg_1
  {
    out << "arg_1: ";
    rosidl_generator_traits::value_to_yaml(msg.arg_1, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Move & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: arg_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arg_1: ";
    rosidl_generator_traits::value_to_yaml(msg.arg_1, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Move & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dora_msgs

namespace rosidl_generator_traits
{

[[deprecated("use dora_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dora_msgs::msg::Move & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dora_msgs::msg::Move & msg)
{
  return dora_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dora_msgs::msg::Move>()
{
  return "dora_msgs::msg::Move";
}

template<>
inline const char * name<dora_msgs::msg::Move>()
{
  return "dora_msgs/msg/Move";
}

template<>
struct has_fixed_size<dora_msgs::msg::Move>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dora_msgs::msg::Move>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dora_msgs::msg::Move>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DORA_MSGS__MSG__DETAIL__MOVE__TRAITS_HPP_
