// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgEkfEuler.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_EKF_EULER__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_EKF_EULER__STRUCT_H_

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
// Member 'angle'
// Member 'accuracy'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'status'
#include "sbg_driver/msg/detail/sbg_ekf_status__struct.h"

// Struct defined in msg/SbgEkfEuler in the package sbg_driver.
typedef struct sbg_driver__msg__SbgEkfEuler
{
  std_msgs__msg__Header header;
  uint32_t time_stamp;
  geometry_msgs__msg__Vector3 angle;
  geometry_msgs__msg__Vector3 accuracy;
  sbg_driver__msg__SbgEkfStatus status;
} sbg_driver__msg__SbgEkfEuler;

// Struct for a sequence of sbg_driver__msg__SbgEkfEuler.
typedef struct sbg_driver__msg__SbgEkfEuler__Sequence
{
  sbg_driver__msg__SbgEkfEuler * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgEkfEuler__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_EKF_EULER__STRUCT_H_
