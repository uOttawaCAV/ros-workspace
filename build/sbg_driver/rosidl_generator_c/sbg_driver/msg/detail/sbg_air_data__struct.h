// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgAirData.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_AIR_DATA__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_AIR_DATA__STRUCT_H_

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
// Member 'status'
#include "sbg_driver/msg/detail/sbg_air_data_status__struct.h"

// Struct defined in msg/SbgAirData in the package sbg_driver.
typedef struct sbg_driver__msg__SbgAirData
{
  std_msgs__msg__Header header;
  uint32_t time_stamp;
  sbg_driver__msg__SbgAirDataStatus status;
  double pressure_abs;
  double altitude;
  double pressure_diff;
  double true_air_speed;
  double air_temperature;
} sbg_driver__msg__SbgAirData;

// Struct for a sequence of sbg_driver__msg__SbgAirData.
typedef struct sbg_driver__msg__SbgAirData__Sequence
{
  sbg_driver__msg__SbgAirData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgAirData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_AIR_DATA__STRUCT_H_
