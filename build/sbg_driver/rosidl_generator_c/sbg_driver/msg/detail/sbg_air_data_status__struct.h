// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgAirDataStatus.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_AIR_DATA_STATUS__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_AIR_DATA_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgAirDataStatus in the package sbg_driver.
typedef struct sbg_driver__msg__SbgAirDataStatus
{
  bool is_delay_time;
  bool pressure_valid;
  bool altitude_valid;
  bool pressure_diff_valid;
  bool air_speed_valid;
  bool air_temperature_valid;
} sbg_driver__msg__SbgAirDataStatus;

// Struct for a sequence of sbg_driver__msg__SbgAirDataStatus.
typedef struct sbg_driver__msg__SbgAirDataStatus__Sequence
{
  sbg_driver__msg__SbgAirDataStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgAirDataStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_AIR_DATA_STATUS__STRUCT_H_
