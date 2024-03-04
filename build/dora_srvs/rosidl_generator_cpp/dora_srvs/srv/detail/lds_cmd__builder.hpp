// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_srvs:srv/LdsCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__LDS_CMD__BUILDER_HPP_
#define DORA_SRVS__SRV__DETAIL__LDS_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_srvs/srv/detail/lds_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_LdsCmd_Request_scan
{
public:
  Init_LdsCmd_Request_scan()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_srvs::srv::LdsCmd_Request scan(::dora_srvs::srv::LdsCmd_Request::_scan_type arg)
  {
    msg_.scan = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::LdsCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::LdsCmd_Request>()
{
  return dora_srvs::srv::builder::Init_LdsCmd_Request_scan();
}

}  // namespace dora_srvs


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_LdsCmd_Response_status
{
public:
  Init_LdsCmd_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_srvs::srv::LdsCmd_Response status(::dora_srvs::srv::LdsCmd_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::LdsCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::LdsCmd_Response>()
{
  return dora_srvs::srv::builder::Init_LdsCmd_Response_status();
}

}  // namespace dora_srvs

#endif  // DORA_SRVS__SRV__DETAIL__LDS_CMD__BUILDER_HPP_
