// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dora_srvs:srv/JobCmd.idl
// generated code does not contain a copyright notice

#ifndef DORA_SRVS__SRV__DETAIL__JOB_CMD__BUILDER_HPP_
#define DORA_SRVS__SRV__DETAIL__JOB_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dora_srvs/srv/detail/job_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_JobCmd_Request_job
{
public:
  Init_JobCmd_Request_job()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_srvs::srv::JobCmd_Request job(::dora_srvs::srv::JobCmd_Request::_job_type arg)
  {
    msg_.job = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::JobCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::JobCmd_Request>()
{
  return dora_srvs::srv::builder::Init_JobCmd_Request_job();
}

}  // namespace dora_srvs


namespace dora_srvs
{

namespace srv
{

namespace builder
{

class Init_JobCmd_Response_status
{
public:
  Init_JobCmd_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dora_srvs::srv::JobCmd_Response status(::dora_srvs::srv::JobCmd_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dora_srvs::srv::JobCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dora_srvs::srv::JobCmd_Response>()
{
  return dora_srvs::srv::builder::Init_JobCmd_Response_status();
}

}  // namespace dora_srvs

#endif  // DORA_SRVS__SRV__DETAIL__JOB_CMD__BUILDER_HPP_
