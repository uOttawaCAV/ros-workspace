// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgStatusGeneral.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_STATUS_GENERAL__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_STATUS_GENERAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgStatusGeneral in the package sbg_driver.
typedef struct sbg_driver__msg__SbgStatusGeneral
{
  bool main_power;
  bool imu_power;
  bool gps_power;
  bool settings;
  bool temperature;
} sbg_driver__msg__SbgStatusGeneral;

// Struct for a sequence of sbg_driver__msg__SbgStatusGeneral.
typedef struct sbg_driver__msg__SbgStatusGeneral__Sequence
{
  sbg_driver__msg__SbgStatusGeneral * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgStatusGeneral__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_STATUS_GENERAL__STRUCT_H_
