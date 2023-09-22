// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sbg_driver:msg/SbgUtcTime.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME__TRAITS_HPP_
#define SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME__TRAITS_HPP_

#include "sbg_driver/msg/detail/sbg_utc_time__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'clock_status'
#include "sbg_driver/msg/detail/sbg_utc_time_status__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sbg_driver::msg::SbgUtcTime>()
{
  return "sbg_driver::msg::SbgUtcTime";
}

template<>
inline const char * name<sbg_driver::msg::SbgUtcTime>()
{
  return "sbg_driver/msg/SbgUtcTime";
}

template<>
struct has_fixed_size<sbg_driver::msg::SbgUtcTime>
  : std::integral_constant<bool, has_fixed_size<sbg_driver::msg::SbgUtcTimeStatus>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<sbg_driver::msg::SbgUtcTime>
  : std::integral_constant<bool, has_bounded_size<sbg_driver::msg::SbgUtcTimeStatus>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<sbg_driver::msg::SbgUtcTime>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME__TRAITS_HPP_
