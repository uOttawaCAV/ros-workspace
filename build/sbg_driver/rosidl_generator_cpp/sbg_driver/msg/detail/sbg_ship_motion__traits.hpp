// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sbg_driver:msg/SbgShipMotion.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_SHIP_MOTION__TRAITS_HPP_
#define SBG_DRIVER__MSG__DETAIL__SBG_SHIP_MOTION__TRAITS_HPP_

#include "sbg_driver/msg/detail/sbg_ship_motion__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'ship_motion'
// Member 'acceleration'
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'status'
#include "sbg_driver/msg/detail/sbg_ship_motion_status__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sbg_driver::msg::SbgShipMotion>()
{
  return "sbg_driver::msg::SbgShipMotion";
}

template<>
inline const char * name<sbg_driver::msg::SbgShipMotion>()
{
  return "sbg_driver/msg/SbgShipMotion";
}

template<>
struct has_fixed_size<sbg_driver::msg::SbgShipMotion>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<sbg_driver::msg::SbgShipMotionStatus>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<sbg_driver::msg::SbgShipMotion>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<sbg_driver::msg::SbgShipMotionStatus>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<sbg_driver::msg::SbgShipMotion>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_SHIP_MOTION__TRAITS_HPP_
