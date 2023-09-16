// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ouster_srvs:srv/SetConfig.idl
// generated code does not contain a copyright notice

#ifndef OUSTER_SRVS__SRV__DETAIL__SET_CONFIG__TRAITS_HPP_
#define OUSTER_SRVS__SRV__DETAIL__SET_CONFIG__TRAITS_HPP_

#include "ouster_srvs/srv/detail/set_config__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ouster_srvs::srv::SetConfig_Request>()
{
  return "ouster_srvs::srv::SetConfig_Request";
}

template<>
inline const char * name<ouster_srvs::srv::SetConfig_Request>()
{
  return "ouster_srvs/srv/SetConfig_Request";
}

template<>
struct has_fixed_size<ouster_srvs::srv::SetConfig_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ouster_srvs::srv::SetConfig_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ouster_srvs::srv::SetConfig_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ouster_srvs::srv::SetConfig_Response>()
{
  return "ouster_srvs::srv::SetConfig_Response";
}

template<>
inline const char * name<ouster_srvs::srv::SetConfig_Response>()
{
  return "ouster_srvs/srv/SetConfig_Response";
}

template<>
struct has_fixed_size<ouster_srvs::srv::SetConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ouster_srvs::srv::SetConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ouster_srvs::srv::SetConfig_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ouster_srvs::srv::SetConfig>()
{
  return "ouster_srvs::srv::SetConfig";
}

template<>
inline const char * name<ouster_srvs::srv::SetConfig>()
{
  return "ouster_srvs/srv/SetConfig";
}

template<>
struct has_fixed_size<ouster_srvs::srv::SetConfig>
  : std::integral_constant<
    bool,
    has_fixed_size<ouster_srvs::srv::SetConfig_Request>::value &&
    has_fixed_size<ouster_srvs::srv::SetConfig_Response>::value
  >
{
};

template<>
struct has_bounded_size<ouster_srvs::srv::SetConfig>
  : std::integral_constant<
    bool,
    has_bounded_size<ouster_srvs::srv::SetConfig_Request>::value &&
    has_bounded_size<ouster_srvs::srv::SetConfig_Response>::value
  >
{
};

template<>
struct is_service<ouster_srvs::srv::SetConfig>
  : std::true_type
{
};

template<>
struct is_service_request<ouster_srvs::srv::SetConfig_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ouster_srvs::srv::SetConfig_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // OUSTER_SRVS__SRV__DETAIL__SET_CONFIG__TRAITS_HPP_
