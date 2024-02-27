// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_msgs:msg/Toys.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__TOYS__TRAITS_HPP_
#define DORA_MSGS__MSG__DETAIL__TOYS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_msgs/msg/detail/toys__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'toys'
#include "dora_msgs/msg/detail/toy__traits.hpp"

namespace dora_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Toys & msg,
  std::ostream & out)
{
  out << "{";
  // member: toys
  {
    if (msg.toys.size() == 0) {
      out << "toys: []";
    } else {
      out << "toys: [";
      size_t pending_items = msg.toys.size();
      for (auto item : msg.toys) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toys & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: toys
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.toys.size() == 0) {
      out << "toys: []\n";
    } else {
      out << "toys:\n";
      for (auto item : msg.toys) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toys & msg, bool use_flow_style = false)
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
  const dora_msgs::msg::Toys & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dora_msgs::msg::Toys & msg)
{
  return dora_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dora_msgs::msg::Toys>()
{
  return "dora_msgs::msg::Toys";
}

template<>
inline const char * name<dora_msgs::msg::Toys>()
{
  return "dora_msgs/msg/Toys";
}

template<>
struct has_fixed_size<dora_msgs::msg::Toys>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dora_msgs::msg::Toys>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dora_msgs::msg::Toys>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DORA_MSGS__MSG__DETAIL__TOYS__TRAITS_HPP_
