// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgUtcTime.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME__STRUCT_H_

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
// Member 'clock_status'
#include "sbg_driver/msg/detail/sbg_utc_time_status__struct.h"

// Struct defined in msg/SbgUtcTime in the package sbg_driver.
typedef struct sbg_driver__msg__SbgUtcTime
{
  std_msgs__msg__Header header;
  uint32_t time_stamp;
  sbg_driver__msg__SbgUtcTimeStatus clock_status;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint32_t nanosec;
  uint32_t gps_tow;
} sbg_driver__msg__SbgUtcTime;

// Struct for a sequence of sbg_driver__msg__SbgUtcTime.
typedef struct sbg_driver__msg__SbgUtcTime__Sequence
{
  sbg_driver__msg__SbgUtcTime * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgUtcTime__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME__STRUCT_H_
