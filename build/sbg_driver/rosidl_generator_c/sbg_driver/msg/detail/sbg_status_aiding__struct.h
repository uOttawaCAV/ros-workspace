// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgStatusAiding.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_STATUS_AIDING__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_STATUS_AIDING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgStatusAiding in the package sbg_driver.
typedef struct sbg_driver__msg__SbgStatusAiding
{
  bool gps1_pos_recv;
  bool gps1_vel_recv;
  bool gps1_hdt_recv;
  bool gps1_utc_recv;
  bool mag_recv;
  bool odo_recv;
  bool dvl_recv;
} sbg_driver__msg__SbgStatusAiding;

// Struct for a sequence of sbg_driver__msg__SbgStatusAiding.
typedef struct sbg_driver__msg__SbgStatusAiding__Sequence
{
  sbg_driver__msg__SbgStatusAiding * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgStatusAiding__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_STATUS_AIDING__STRUCT_H_
