// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_srvs:srv/WheelsCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__WHEELS_CMD__BUILDER_HPP_
#define DORA_SRVS__SRV__DETAIL__WHEELS_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_srvs/srv/detail/wheels_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_WheelsCmd_Request_magnitude
{
public:
  explicit Init_WheelsCmd_Request_magnitude(::dora_srvs::srv::WheelsCmd_Request & msg)
  : msg_(msg)
  {}
  ::dora_srvs::srv::WheelsCmd_Request magnitude(::dora_srvs::srv::WheelsCmd_Request::_magnitude_type arg)
  {
    msg_.magnitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::WheelsCmd_Request msg_;
};

class Init_WheelsCmd_Request_type
{
public:
  Init_WheelsCmd_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelsCmd_Request_magnitude type(::dora_srvs::srv::WheelsCmd_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_WheelsCmd_Request_magnitude(msg_);
  }

private:
  ::dora_srvs::srv::WheelsCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::WheelsCmd_Request>()
{
  return dora_srvs::srv::builder::Init_WheelsCmd_Request_type();
}

}  // namespace dora_srvs


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_WheelsCmd_Response_status
{
public:
  Init_WheelsCmd_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_srvs::srv::WheelsCmd_Response status(::dora_srvs::srv::WheelsCmd_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::WheelsCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::WheelsCmd_Response>()
{
  return dora_srvs::srv::builder::Init_WheelsCmd_Response_status();
}

}  // namespace dora_srvs

#endif  // DORA_SRVS__SRV__DETAIL__WHEELS_CMD__BUILDER_HPP_
