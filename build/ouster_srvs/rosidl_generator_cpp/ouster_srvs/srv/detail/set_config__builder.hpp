// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ouster_srvs:srv/SetConfig.idl
// generated code does not contain a copyright notice

#ifndef OUSTER_SRVS__SRV__DETAIL__SET_CONFIG__BUILDER_HPP_
#define OUSTER_SRVS__SRV__DETAIL__SET_CONFIG__BUILDER_HPP_

#include "ouster_srvs/srv/detail/set_config__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ouster_srvs
{

namespace srv
{

namespace builder
{

class Init_SetConfig_Request_config_file
{
public:
  Init_SetConfig_Request_config_file()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ouster_srvs::srv::SetConfig_Request config_file(::ouster_srvs::srv::SetConfig_Request::_config_file_type arg)
  {
    msg_.config_file = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ouster_srvs::srv::SetConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ouster_srvs::srv::SetConfig_Request>()
{
  return ouster_srvs::srv::builder::Init_SetConfig_Request_config_file();
}

}  // namespace ouster_srvs


namespace ouster_srvs
{

namespace srv
{

namespace builder
{

class Init_SetConfig_Response_config
{
public:
  Init_SetConfig_Response_config()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ouster_srvs::srv::SetConfig_Response config(::ouster_srvs::srv::SetConfig_Response::_config_type arg)
  {
    msg_.config = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ouster_srvs::srv::SetConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ouster_srvs::srv::SetConfig_Response>()
{
  return ouster_srvs::srv::builder::Init_SetConfig_Response_config();
}

}  // namespace ouster_srvs

#endif  // OUSTER_SRVS__SRV__DETAIL__SET_CONFIG__BUILDER_HPP_
