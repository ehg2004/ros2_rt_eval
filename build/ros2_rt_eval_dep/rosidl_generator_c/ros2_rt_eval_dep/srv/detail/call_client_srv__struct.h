// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_rt_eval_dep:srv/CallClientSrv.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__STRUCT_H_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CallClientSrv in the package ros2_rt_eval_dep.
typedef struct ros2_rt_eval_dep__srv__CallClientSrv_Request
{
  int32_t num_calls;
} ros2_rt_eval_dep__srv__CallClientSrv_Request;

// Struct for a sequence of ros2_rt_eval_dep__srv__CallClientSrv_Request.
typedef struct ros2_rt_eval_dep__srv__CallClientSrv_Request__Sequence
{
  ros2_rt_eval_dep__srv__CallClientSrv_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_rt_eval_dep__srv__CallClientSrv_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'latencies'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/CallClientSrv in the package ros2_rt_eval_dep.
typedef struct ros2_rt_eval_dep__srv__CallClientSrv_Response
{
  rosidl_runtime_c__int64__Sequence latencies;
} ros2_rt_eval_dep__srv__CallClientSrv_Response;

// Struct for a sequence of ros2_rt_eval_dep__srv__CallClientSrv_Response.
typedef struct ros2_rt_eval_dep__srv__CallClientSrv_Response__Sequence
{
  ros2_rt_eval_dep__srv__CallClientSrv_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_rt_eval_dep__srv__CallClientSrv_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__STRUCT_H_
