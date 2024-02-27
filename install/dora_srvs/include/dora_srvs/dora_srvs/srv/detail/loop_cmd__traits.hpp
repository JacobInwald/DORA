// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_srvs:srv/LoopCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__LOOP_CMD__TRAITS_HPP_
#define DORA_SRVS__SRV__DETAIL__LOOP_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_srvs/srv/detail/loop_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dora_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoopCmd_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start
  {
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoopCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoopCmd_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dora_srvs

namespace rosidl_generator_traits
{

[[deprecated("use dora_srvs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dora_srvs::srv::LoopCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const dora_srvs::srv::LoopCmd_Request & msg)
{
  return dora_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dora_srvs::srv::LoopCmd_Request>()
{
  return "dora_srvs::srv::LoopCmd_Request";
}

template<>
inline const char * name<dora_srvs::srv::LoopCmd_Request>()
{
  return "dora_srvs/srv/LoopCmd_Request";
}

template<>
struct has_fixed_size<dora_srvs::srv::LoopCmd_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dora_srvs::srv::LoopCmd_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dora_srvs::srv::LoopCmd_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dora_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoopCmd_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoopCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoopCmd_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dora_srvs

namespace rosidl_generator_traits
{

[[deprecated("use dora_srvs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dora_srvs::srv::LoopCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const dora_srvs::srv::LoopCmd_Response & msg)
{
  return dora_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dora_srvs::srv::LoopCmd_Response>()
{
  return "dora_srvs::srv::LoopCmd_Response";
}

template<>
inline const char * name<dora_srvs::srv::LoopCmd_Response>()
{
  return "dora_srvs/srv/LoopCmd_Response";
}

template<>
struct has_fixed_size<dora_srvs::srv::LoopCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dora_srvs::srv::LoopCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dora_srvs::srv::LoopCmd_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dora_srvs::srv::LoopCmd>()
{
  return "dora_srvs::srv::LoopCmd";
}

template<>
inline const char * name<dora_srvs::srv::LoopCmd>()
{
  return "dora_srvs/srv/LoopCmd";
}

template<>
struct has_fixed_size<dora_srvs::srv::LoopCmd>
  : std::integral_constant<
    bool,
    has_fixed_size<dora_srvs::srv::LoopCmd_Request>::value &&
    has_fixed_size<dora_srvs::srv::LoopCmd_Response>::value
  >
{
};

template<>
struct has_bounded_size<dora_srvs::srv::LoopCmd>
  : std::integral_constant<
    bool,
    has_bounded_size<dora_srvs::srv::LoopCmd_Request>::value &&
    has_bounded_size<dora_srvs::srv::LoopCmd_Response>::value
  >
{
};

template<>
struct is_service<dora_srvs::srv::LoopCmd>
  : std::true_type
{
};

template<>
struct is_service_request<dora_srvs::srv::LoopCmd_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dora_srvs::srv::LoopCmd_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DORA_SRVS__SRV__DETAIL__LOOP_CMD__TRAITS_HPP_
