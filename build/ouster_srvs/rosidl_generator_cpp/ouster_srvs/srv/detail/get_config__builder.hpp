// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ouster_srvs:srv/GetConfig.idl
// generated code does not contain a copyright notice

#ifndef OUSTER_SRVS__SRV__DETAIL__GET_CONFIG__BUILDER_HPP_
#define OUSTER_SRVS__SRV__DETAIL__GET_CONFIG__BUILDER_HPP_

#include "ouster_srvs/srv/detail/get_config__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ouster_srvs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ouster_srvs::srv::GetConfig_Request>()
{
  return ::ouster_srvs::srv::GetConfig_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ouster_srvs


namespace ouster_srvs
{

namespace srv
{

namespace builder
{

class Init_GetConfig_Response_config
{
public:
  Init_GetConfig_Response_config()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ouster_srvs::srv::GetConfig_Response config(::ouster_srvs::srv::GetConfig_Response::_config_type arg)
  {
    msg_.config = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ouster_srvs::srv::GetConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ouster_srvs::srv::GetConfig_Response>()
{
  return ouster_srvs::srv::builder::Init_GetConfig_Response_config();
}

}  // namespace ouster_srvs

#endif  // OUSTER_SRVS__SRV__DETAIL__GET_CONFIG__BUILDER_HPP_
