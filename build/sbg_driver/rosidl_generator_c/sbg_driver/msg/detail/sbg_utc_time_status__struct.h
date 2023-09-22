// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgUtcTimeStatus.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME_STATUS__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgUtcTimeStatus in the package sbg_driver.
typedef struct sbg_driver__msg__SbgUtcTimeStatus
{
  bool clock_stable;
  uint8_t clock_status;
  bool clock_utc_sync;
  uint8_t clock_utc_status;
} sbg_driver__msg__SbgUtcTimeStatus;

// Struct for a sequence of sbg_driver__msg__SbgUtcTimeStatus.
typedef struct sbg_driver__msg__SbgUtcTimeStatus__Sequence
{
  sbg_driver__msg__SbgUtcTimeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgUtcTimeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_UTC_TIME_STATUS__STRUCT_H_
