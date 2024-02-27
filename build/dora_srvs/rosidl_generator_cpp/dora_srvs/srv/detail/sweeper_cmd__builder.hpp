// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_srvs:srv/SweeperCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__SWEEPER_CMD__BUILDER_HPP_
#define DORA_SRVS__SRV__DETAIL__SWEEPER_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_srvs/srv/detail/sweeper_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_SweeperCmd_Request_move
{
public:
  Init_SweeperCmd_Request_move()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_srvs::srv::SweeperCmd_Request move(::dora_srvs::srv::SweeperCmd_Request::_move_type arg)
  {
    msg_.move = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::SweeperCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::SweeperCmd_Request>()
{
  return dora_srvs::srv::builder::Init_SweeperCmd_Request_move();
}

}  // namespace dora_srvs


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_SweeperCmd_Response_status
{
public:
  Init_SweeperCmd_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_srvs::srv::SweeperCmd_Response status(::dora_srvs::srv::SweeperCmd_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::SweeperCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::SweeperCmd_Response>()
{
  return dora_srvs::srv::builder::Init_SweeperCmd_Response_status();
}

}  // namespace dora_srvs

#endif  // DORA_SRVS__SRV__DETAIL__SWEEPER_CMD__BUILDER_HPP_
