// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sbg_driver:msg/SbgGpsRaw.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_GPS_RAW__TRAITS_HPP_
#define SBG_DRIVER__MSG__DETAIL__SBG_GPS_RAW__TRAITS_HPP_

#include "sbg_driver/msg/detail/sbg_gps_raw__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sbg_driver::msg::SbgGpsRaw>()
{
  return "sbg_driver::msg::SbgGpsRaw";
}

template<>
inline const char * name<sbg_driver::msg::SbgGpsRaw>()
{
  return "sbg_driver/msg/SbgGpsRaw";
}

template<>
struct has_fixed_size<sbg_driver::msg::SbgGpsRaw>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sbg_driver::msg::SbgGpsRaw>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sbg_driver::msg::SbgGpsRaw>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_GPS_RAW__TRAITS_HPP_
