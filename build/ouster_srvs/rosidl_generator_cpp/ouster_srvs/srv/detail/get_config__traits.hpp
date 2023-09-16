// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ouster_srvs:srv/GetConfig.idl
// generated code does not contain a copyright notice

#ifndef OUSTER_SRVS__SRV__DETAIL__GET_CONFIG__TRAITS_HPP_
#define OUSTER_SRVS__SRV__DETAIL__GET_CONFIG__TRAITS_HPP_

#include "ouster_srvs/srv/detail/get_config__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ouster_srvs::srv::GetConfig_Request>()
{
  return "ouster_srvs::srv::GetConfig_Request";
}

template<>
inline const char * name<ouster_srvs::srv::GetConfig_Request>()
{
  return "ouster_srvs/srv/GetConfig_Request";
}

template<>
struct has_fixed_size<ouster_srvs::srv::GetConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ouster_srvs::srv::GetConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ouster_srvs::srv::GetConfig_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ouster_srvs::srv::GetConfig_Response>()
{
  return "ouster_srvs::srv::GetConfig_Response";
}

template<>
inline const char * name<ouster_srvs::srv::GetConfig_Response>()
{
  return "ouster_srvs/srv/GetConfig_Response";
}

template<>
struct has_fixed_size<ouster_srvs::srv::GetConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ouster_srvs::srv::GetConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ouster_srvs::srv::GetConfig_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ouster_srvs::srv::GetConfig>()
{
  return "ouster_srvs::srv::GetConfig";
}

template<>
inline const char * name<ouster_srvs::srv::GetConfig>()
{
  return "ouster_srvs/srv/GetConfig";
}

template<>
struct has_fixed_size<ouster_srvs::srv::GetConfig>
  : std::integral_constant<
    bool,
    has_fixed_size<ouster_srvs::srv::GetConfig_Request>::value &&
    has_fixed_size<ouster_srvs::srv::GetConfig_Response>::value
  >
{
};

template<>
struct has_bounded_size<ouster_srvs::srv::GetConfig>
  : std::integral_constant<
    bool,
    has_bounded_size<ouster_srvs::srv::GetConfig_Request>::value &&
    has_bounded_size<ouster_srvs::srv::GetConfig_Response>::value
  >
{
};

template<>
struct is_service<ouster_srvs::srv::GetConfig>
  : std::true_type
{
};

template<>
struct is_service_request<ouster_srvs::srv::GetConfig_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ouster_srvs::srv::GetConfig_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // OUSTER_SRVS__SRV__DETAIL__GET_CONFIG__TRAITS_HPP_
