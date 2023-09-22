// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgGpsPosStatus.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_GPS_POS_STATUS__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_GPS_POS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgGpsPosStatus in the package sbg_driver.
typedef struct sbg_driver__msg__SbgGpsPosStatus
{
  uint8_t status;
  uint8_t type;
  bool gps_l1_used;
  bool gps_l2_used;
  bool gps_l5_used;
  bool glo_l1_used;
  bool glo_l2_used;
} sbg_driver__msg__SbgGpsPosStatus;

// Struct for a sequence of sbg_driver__msg__SbgGpsPosStatus.
typedef struct sbg_driver__msg__SbgGpsPosStatus__Sequence
{
  sbg_driver__msg__SbgGpsPosStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgGpsPosStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_GPS_POS_STATUS__STRUCT_H_
