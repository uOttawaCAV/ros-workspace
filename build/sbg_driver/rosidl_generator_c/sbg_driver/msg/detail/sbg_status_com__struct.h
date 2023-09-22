// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgStatusCom.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_STATUS_COM__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_STATUS_COM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgStatusCom in the package sbg_driver.
typedef struct sbg_driver__msg__SbgStatusCom
{
  bool port_a;
  bool port_b;
  bool port_c;
  bool port_d;
  bool port_e;
  bool port_a_rx;
  bool port_a_tx;
  bool port_b_rx;
  bool port_b_tx;
  bool port_c_rx;
  bool port_c_tx;
  bool port_d_rx;
  bool port_d_tx;
  bool port_e_rx;
  bool port_e_tx;
  bool can_rx;
  bool can_tx;
  uint8_t can_status;
} sbg_driver__msg__SbgStatusCom;

// Struct for a sequence of sbg_driver__msg__SbgStatusCom.
typedef struct sbg_driver__msg__SbgStatusCom__Sequence
{
  sbg_driver__msg__SbgStatusCom * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgStatusCom__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_STATUS_COM__STRUCT_H_
