// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_msgs:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef DORA_MSGS__MSG__DETAIL__MAP__TRAITS_HPP_
#define DORA_MSGS__MSG__DETAIL__MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_msgs/msg/detail/map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'offset'
#include "dora_msgs/msg/detail/pose__traits.hpp"
// Member 'clouds'
#include "dora_msgs/msg/detail/cloud__traits.hpp"

namespace dora_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Map & msg,
  std::ostream & out)
{
  out << "{";
  // member: offset
  {
    out << "offset: ";
    to_flow_style_yaml(msg.offset, out);
    out << ", ";
  }

  // member: clouds
  {
    if (msg.clouds.size() == 0) {
      out << "clouds: []";
    } else {
      out << "clouds: [";
      size_t pending_items = msg.clouds.size();
      for (auto item : msg.clouds) {
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
  const Map & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset:\n";
    to_block_style_yaml(msg.offset, out, indentation + 2);
  }

  // member: clouds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.clouds.size() == 0) {
      out << "clouds: []\n";
    } else {
      out << "clouds:\n";
      for (auto item : msg.clouds) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Map & msg, bool use_flow_style = false)
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
  const dora_msgs::msg::Map & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dora_msgs::msg::Map & msg)
{
  return dora_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dora_msgs::msg::Map>()
{
  return "dora_msgs::msg::Map";
}

template<>
inline const char * name<dora_msgs::msg::Map>()
{
  return "dora_msgs/msg/Map";
}

template<>
struct has_fixed_size<dora_msgs::msg::Map>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dora_msgs::msg::Map>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dora_msgs::msg::Map>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DORA_MSGS__MSG__DETAIL__MAP__TRAITS_HPP_
