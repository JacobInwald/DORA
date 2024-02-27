// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dora_srvs:srv/WheelsCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__WHEELS_CMD__TRAITS_HPP_
#define DORA_SRVS__SRV__DETAIL__WHEELS_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dora_srvs/srv/detail/wheels_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dora_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WheelsCmd_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: magnitude
  {
    out << "magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.magnitude, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WheelsCmd_Request & msg,
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

  // member: magnitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.magnitude, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WheelsCmd_Request & msg, bool use_flow_style = false)
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
  const dora_srvs::srv::WheelsCmd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const dora_srvs::srv::WheelsCmd_Request & msg)
{
  return dora_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dora_srvs::srv::WheelsCmd_Request>()
{
  return "dora_srvs::srv::WheelsCmd_Request";
}

template<>
inline const char * name<dora_srvs::srv::WheelsCmd_Request>()
{
  return "dora_srvs/srv/WheelsCmd_Request";
}

template<>
struct has_fixed_size<dora_srvs::srv::WheelsCmd_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dora_srvs::srv::WheelsCmd_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dora_srvs::srv::WheelsCmd_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dora_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WheelsCmd_Response & msg,
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
  const WheelsCmd_Response & msg,
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

inline std::string to_yaml(const WheelsCmd_Response & msg, bool use_flow_style = false)
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
  const dora_srvs::srv::WheelsCmd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dora_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dora_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const dora_srvs::srv::WheelsCmd_Response & msg)
{
  return dora_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dora_srvs::srv::WheelsCmd_Response>()
{
  return "dora_srvs::srv::WheelsCmd_Response";
}

template<>
inline const char * name<dora_srvs::srv::WheelsCmd_Response>()
{
  return "dora_srvs/srv/WheelsCmd_Response";
}

template<>
struct has_fixed_size<dora_srvs::srv::WheelsCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dora_srvs::srv::WheelsCmd_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dora_srvs::srv::WheelsCmd_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dora_srvs::srv::WheelsCmd>()
{
  return "dora_srvs::srv::WheelsCmd";
}

template<>
inline const char * name<dora_srvs::srv::WheelsCmd>()
{
  return "dora_srvs/srv/WheelsCmd";
}

template<>
struct has_fixed_size<dora_srvs::srv::WheelsCmd>
  : std::integral_constant<
    bool,
    has_fixed_size<dora_srvs::srv::WheelsCmd_Request>::value &&
    has_fixed_size<dora_srvs::srv::WheelsCmd_Response>::value
  >
{
};

template<>
struct has_bounded_size<dora_srvs::srv::WheelsCmd>
  : std::integral_constant<
    bool,
    has_bounded_size<dora_srvs::srv::WheelsCmd_Request>::value &&
    has_bounded_size<dora_srvs::srv::WheelsCmd_Response>::value
  >
{
};

template<>
struct is_service<dora_srvs::srv::WheelsCmd>
  : std::true_type
{
};

template<>
struct is_service_request<dora_srvs::srv::WheelsCmd_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dora_srvs::srv::WheelsCmd_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DORA_SRVS__SRV__DETAIL__WHEELS_CMD__TRAITS_HPP_
