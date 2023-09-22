// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgEkfStatus.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_EKF_STATUS__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_EKF_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgEkfStatus in the package sbg_driver.
typedef struct sbg_driver__msg__SbgEkfStatus
{
  uint8_t solution_mode;
  bool attitude_valid;
  bool heading_valid;
  bool velocity_valid;
  bool position_valid;
  bool vert_ref_used;
  bool mag_ref_used;
  bool gps1_vel_used;
  bool gps1_pos_used;
  bool gps1_course_used;
  bool gps1_hdt_used;
  bool gps2_vel_used;
  bool gps2_pos_used;
  bool gps2_course_used;
  bool gps2_hdt_used;
  bool odo_used;
} sbg_driver__msg__SbgEkfStatus;

// Struct for a sequence of sbg_driver__msg__SbgEkfStatus.
typedef struct sbg_driver__msg__SbgEkfStatus__Sequence
{
  sbg_driver__msg__SbgEkfStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgEkfStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_EKF_STATUS__STRUCT_H_
