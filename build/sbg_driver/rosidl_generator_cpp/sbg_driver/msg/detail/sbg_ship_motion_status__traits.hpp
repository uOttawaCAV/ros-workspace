// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sbg_driver:msg/SbgShipMotionStatus.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_SHIP_MOTION_STATUS__TRAITS_HPP_
#define SBG_DRIVER__MSG__DETAIL__SBG_SHIP_MOTION_STATUS__TRAITS_HPP_

#include "sbg_driver/msg/detail/sbg_ship_motion_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sbg_driver::msg::SbgShipMotionStatus>()
{
  return "sbg_driver::msg::SbgShipMotionStatus";
}

template<>
inline const char * name<sbg_driver::msg::SbgShipMotionStatus>()
{
  return "sbg_driver/msg/SbgShipMotionStatus";
}

template<>
struct has_fixed_size<sbg_driver::msg::SbgShipMotionStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sbg_driver::msg::SbgShipMotionStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sbg_driver::msg::SbgShipMotionStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_SHIP_MOTION_STATUS__TRAITS_HPP_
