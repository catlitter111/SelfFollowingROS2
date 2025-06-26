// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dlrobot_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice

#ifndef DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__STRUCT_H_
#define DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Data in the package dlrobot_robot_msg.
typedef struct dlrobot_robot_msg__msg__Data
{
  float x;
  float y;
  float z;
} dlrobot_robot_msg__msg__Data;

// Struct for a sequence of dlrobot_robot_msg__msg__Data.
typedef struct dlrobot_robot_msg__msg__Data__Sequence
{
  dlrobot_robot_msg__msg__Data * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dlrobot_robot_msg__msg__Data__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DLROBOT_ROBOT_MSG__MSG__DETAIL__DATA__STRUCT_H_
