// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgEvent.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_EVENT__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_EVENT__STRUCT_H_

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

// Struct defined in msg/SbgEvent in the package sbg_driver.
typedef struct sbg_driver__msg__SbgEvent
{
  std_msgs__msg__Header header;
  uint32_t time_stamp;
  bool overflow;
  bool offset_0_valid;
  bool offset_1_valid;
  bool offset_2_valid;
  bool offset_3_valid;
  uint16_t time_offset_0;
  uint16_t time_offset_1;
  uint16_t time_offset_2;
  uint16_t time_offset_3;
} sbg_driver__msg__SbgEvent;

// Struct for a sequence of sbg_driver__msg__SbgEvent.
typedef struct sbg_driver__msg__SbgEvent__Sequence
{
  sbg_driver__msg__SbgEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_EVENT__STRUCT_H_
