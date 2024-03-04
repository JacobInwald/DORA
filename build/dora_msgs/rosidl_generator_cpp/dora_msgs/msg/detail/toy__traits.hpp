// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_msgs:msg/Toy.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOY__TRAITS_HPP_
#define DORA_MSGS__MSG__DETAIL__TOY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_msgs/msg/detail/toy__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "dora_msgs/msg/detail/pose__traits.hpp"

namespace dora_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Toy & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: cls
  {
    out << "cls: ";
    rosidl_generator_traits::value_to_yaml(msg.cls, out);
    out << ", ";
  }

  // member: conf
  {
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toy & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: cls
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cls: ";
    rosidl_generator_traits::value_to_yaml(msg.cls, out);
    out << "\n";
  }

  // member: conf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toy & msg, bool use_flow_style = false)
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
  const dora_msgs::msg::Toy & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dora_msgs::msg::Toy & msg)
{
  return dora_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dora_msgs::msg::Toy>()
{
  return "dora_msgs::msg::Toy";
}

template<>
inline const char * name<dora_msgs::msg::Toy>()
{
  return "dora_msgs/msg/Toy";
}

template<>
struct has_fixed_size<dora_msgs::msg::Toy>
  : std::integral_constant<bool, has_fixed_size<dora_msgs::msg::Pose>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<dora_msgs::msg::Toy>
  : std::integral_constant<bool, has_bounded_size<dora_msgs::msg::Pose>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<dora_msgs::msg::Toy>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DORA_MSGS__MSG__DETAIL__TOY__TRAITS_HPP_
