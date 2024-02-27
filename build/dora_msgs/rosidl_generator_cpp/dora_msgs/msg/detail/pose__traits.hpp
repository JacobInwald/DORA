// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__POSE__TRAITS_HPP_
#define DORA_MSGS__MSG__DETAIL__POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_msgs/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dora_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Pose & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: rot
  {
    out << "rot: ";
    rosidl_generator_traits::value_to_yaml(msg.rot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: rot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rot: ";
    rosidl_generator_traits::value_to_yaml(msg.rot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pose & msg, bool use_flow_style = false)
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
  const dora_msgs::msg::Pose & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dora_msgs::msg::Pose & msg)
{
  return dora_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dora_msgs::msg::Pose>()
{
  return "dora_msgs::msg::Pose";
}

template<>
inline const char * name<dora_msgs::msg::Pose>()
{
  return "dora_msgs/msg/Pose";
}

template<>
struct has_fixed_size<dora_msgs::msg::Pose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dora_msgs::msg::Pose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dora_msgs::msg::Pose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DORA_MSGS__MSG__DETAIL__POSE__TRAITS_HPP_
