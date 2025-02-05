// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_rt_eval_dep:srv/Vector.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__STRUCT_H_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'input_vector'
// Member 'client_id_vector'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/Vector in the package ros2_rt_eval_dep.
typedef struct ros2_rt_eval_dep__srv__Vector_Request
{
  /// The vector to be operated on
  rosidl_runtime_c__int16__Sequence input_vector;
  /// The client ID vector
  rosidl_runtime_c__int16__Sequence client_id_vector;
} ros2_rt_eval_dep__srv__Vector_Request;

// Struct for a sequence of ros2_rt_eval_dep__srv__Vector_Request.
typedef struct ros2_rt_eval_dep__srv__Vector_Request__Sequence
{
  ros2_rt_eval_dep__srv__Vector_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_rt_eval_dep__srv__Vector_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'output_vector'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/Vector in the package ros2_rt_eval_dep.
typedef struct ros2_rt_eval_dep__srv__Vector_Response
{
  /// The resulting output vector after the operation
  rosidl_runtime_c__int16__Sequence output_vector;
  /// The duration of the operation
  int64_t duration;
  int64_t t2;
  int64_t t3;
} ros2_rt_eval_dep__srv__Vector_Response;

// Struct for a sequence of ros2_rt_eval_dep__srv__Vector_Response.
typedef struct ros2_rt_eval_dep__srv__Vector_Response__Sequence
{
  ros2_rt_eval_dep__srv__Vector_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_rt_eval_dep__srv__Vector_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__STRUCT_H_
