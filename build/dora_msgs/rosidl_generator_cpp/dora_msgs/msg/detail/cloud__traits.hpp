// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_msgs:msg/Cloud.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__CLOUD__TRAITS_HPP_
#define DORA_MSGS__MSG__DETAIL__CLOUD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_msgs/msg/detail/cloud__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "dora_msgs/msg/detail/pose__traits.hpp"
// Member 'scan'
#include "sensor_msgs/msg/detail/laser_scan__traits.hpp"

namespace dora_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Cloud & msg,
  std::ostream & out)
{
  out << "{";
  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: scan
  {
    out << "scan: ";
    to_flow_style_yaml(msg.scan, out);
    out << ", ";
  }

  // member: max_range
  {
    out << "max_range: ";
    rosidl_generator_traits::value_to_yaml(msg.max_range, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Cloud & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: scan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scan:\n";
    to_block_style_yaml(msg.scan, out, indentation + 2);
  }

  // member: max_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_range: ";
    rosidl_generator_traits::value_to_yaml(msg.max_range, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Cloud & msg, bool use_flow_style = false)
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
  const dora_msgs::msg::Cloud & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dora_msgs::msg::Cloud & msg)
{
  return dora_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dora_msgs::msg::Cloud>()
{
  return "dora_msgs::msg::Cloud";
}

template<>
inline const char * name<dora_msgs::msg::Cloud>()
{
  return "dora_msgs/msg/Cloud";
}

template<>
struct has_fixed_size<dora_msgs::msg::Cloud>
  : std::integral_constant<bool, has_fixed_size<dora_msgs::msg::Pose>::value && has_fixed_size<sensor_msgs::msg::LaserScan>::value> {};

template<>
struct has_bounded_size<dora_msgs::msg::Cloud>
  : std::integral_constant<bool, has_bounded_size<dora_msgs::msg::Pose>::value && has_bounded_size<sensor_msgs::msg::LaserScan>::value> {};

template<>
struct is_message<dora_msgs::msg::Cloud>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DORA_MSGS__MSG__DETAIL__CLOUD__TRAITS_HPP_
