// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sbg_driver:msg/SbgImuStatus.idl
// generated code does not contain a copyright notice

#ifndef SBG_DRIVER__MSG__DETAIL__SBG_IMU_STATUS__STRUCT_H_
#define SBG_DRIVER__MSG__DETAIL__SBG_IMU_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SbgImuStatus in the package sbg_driver.
typedef struct sbg_driver__msg__SbgImuStatus
{
  bool imu_com;
  bool imu_status;
  bool imu_accel_x;
  bool imu_accel_y;
  bool imu_accel_z;
  bool imu_gyro_x;
  bool imu_gyro_y;
  bool imu_gyro_z;
  bool imu_accels_in_range;
  bool imu_gyros_in_range;
} sbg_driver__msg__SbgImuStatus;

// Struct for a sequence of sbg_driver__msg__SbgImuStatus.
typedef struct sbg_driver__msg__SbgImuStatus__Sequence
{
  sbg_driver__msg__SbgImuStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sbg_driver__msg__SbgImuStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SBG_DRIVER__MSG__DETAIL__SBG_IMU_STATUS__STRUCT_H_
