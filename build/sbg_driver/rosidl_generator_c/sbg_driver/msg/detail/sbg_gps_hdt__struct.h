// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgGpsHdt.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_GPS_HDT__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_GPS_HDT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/SbgGpsHdt in the package sbg_driver.
typedef struct sbg_driver__msg__SbgGpsHdt
{
  std_msgs__msg__Header header;
  uint32_t time_stamp;
  uint16_t status;
  uint32_t tow;
  float true_heading;
  float true_heading_acc;
  float pitch;
  float pitch_acc;
  float baseline;
} sbg_driver__msg__SbgGpsHdt;

// Struct for a sequence of sbg_driver__msg__SbgGpsHdt.
typedef struct sbg_driver__msg__SbgGpsHdt__Sequence
{
  sbg_driver__msg__SbgGpsHdt * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgGpsHdt__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_GPS_HDT__STRUCT_H_
