// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ouster_srvs:srv/GetMetadata.idl
// generated code does not contain a copyright notice

#ifndef OUSTER_SRVS__SRV__DETAIL__GET_METADATA__BUILDER_HPP_
#define OUSTER_SRVS__SRV__DETAIL__GET_METADATA__BUILDER_HPP_

#include "ouster_srvs/srv/detail/get_metadata__struct.hpp"
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
auto build<::ouster_srvs::srv::GetMetadata_Request>()
{
  return ::ouster_srvs::srv::GetMetadata_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ouster_srvs


namespace ouster_srvs
{

namespace srv
{

namespace builder
{

class Init_GetMetadata_Response_metadata
{
public:
  Init_GetMetadata_Response_metadata()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ouster_srvs::srv::GetMetadata_Response metadata(::ouster_srvs::srv::GetMetadata_Response::_metadata_type arg)
  {
    msg_.metadata = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ouster_srvs::srv::GetMetadata_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ouster_srvs::srv::GetMetadata_Response>()
{
  return ouster_srvs::srv::builder::Init_GetMetadata_Response_metadata();
}

}  // namespace ouster_srvs

#endif  // OUSTER_SRVS__SRV__DETAIL__GET_METADATA__BUILDER_HPP_
