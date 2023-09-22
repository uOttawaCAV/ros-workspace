// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sbg_driver:msg/SbgStatus.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_STATUS__TRAITS_HPP_
#define SBG_DRIVER__MSG__DETAIL__SBG_STATUS__TRAITS_HPP_

#include "sbg_driver/msg/detail/sbg_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'status_general'
#include "sbg_driver/msg/detail/sbg_status_general__traits.hpp"
// Member 'status_com'
#include "sbg_driver/msg/detail/sbg_status_com__traits.hpp"
// Member 'status_aiding'
#include "sbg_driver/msg/detail/sbg_status_aiding__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sbg_driver::msg::SbgStatus>()
{
  return "sbg_driver::msg::SbgStatus";
}

template<>
inline const char * name<sbg_driver::msg::SbgStatus>()
{
  return "sbg_driver/msg/SbgStatus";
}

template<>
struct has_fixed_size<sbg_driver::msg::SbgStatus>
  : std::integral_constant<bool, has_fixed_size<sbg_driver::msg::SbgStatusAiding>::value && has_fixed_size<sbg_driver::msg::SbgStatusCom>::value && has_fixed_size<sbg_driver::msg::SbgStatusGeneral>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<sbg_driver::msg::SbgStatus>
  : std::integral_constant<bool, has_bounded_size<sbg_driver::msg::SbgStatusAiding>::value && has_bounded_size<sbg_driver::msg::SbgStatusCom>::value && has_bounded_size<sbg_driver::msg::SbgStatusGeneral>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<sbg_driver::msg::SbgStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_STATUS__TRAITS_HPP_
