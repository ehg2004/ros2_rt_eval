// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2_rt_eval_dep:srv/Vector.idl
// generated code does not contain a copyright notice
#include "ros2_rt_eval_dep/srv/detail/vector__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2_rt_eval_dep/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2_rt_eval_dep/srv/detail/vector__struct.h"
#include "ros2_rt_eval_dep/srv/detail/vector__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // client_id_vector, input_vector
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // client_id_vector, input_vector

// forward declare type support functions


using _Vector_Request__ros_msg_type = ros2_rt_eval_dep__srv__Vector_Request;

static bool _Vector_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Vector_Request__ros_msg_type * ros_message = static_cast<const _Vector_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: input_vector
  {
    size_t size = ros_message->input_vector.size;
    auto array_ptr = ros_message->input_vector.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: client_id_vector
  {
    size_t size = ros_message->client_id_vector.size;
    auto array_ptr = ros_message->client_id_vector.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Vector_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Vector_Request__ros_msg_type * ros_message = static_cast<_Vector_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: input_vector
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->input_vector.data) {
      rosidl_runtime_c__int16__Sequence__fini(&ros_message->input_vector);
    }
    if (!rosidl_runtime_c__int16__Sequence__init(&ros_message->input_vector, size)) {
      fprintf(stderr, "failed to create array for field 'input_vector'");
      return false;
    }
    auto array_ptr = ros_message->input_vector.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: client_id_vector
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->client_id_vector.data) {
      rosidl_runtime_c__int16__Sequence__fini(&ros_message->client_id_vector);
    }
    if (!rosidl_runtime_c__int16__Sequence__init(&ros_message->client_id_vector, size)) {
      fprintf(stderr, "failed to create array for field 'client_id_vector'");
      return false;
    }
    auto array_ptr = ros_message->client_id_vector.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_rt_eval_dep
size_t get_serialized_size_ros2_rt_eval_dep__srv__Vector_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Vector_Request__ros_msg_type * ros_message = static_cast<const _Vector_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name input_vector
  {
    size_t array_size = ros_message->input_vector.size;
    auto array_ptr = ros_message->input_vector.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name client_id_vector
  {
    size_t array_size = ros_message->client_id_vector.size;
    auto array_ptr = ros_message->client_id_vector.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Vector_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2_rt_eval_dep__srv__Vector_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_rt_eval_dep
size_t max_serialized_size_ros2_rt_eval_dep__srv__Vector_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: input_vector
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: client_id_vector
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2_rt_eval_dep__srv__Vector_Request;
    is_plain =
      (
      offsetof(DataType, client_id_vector) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Vector_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2_rt_eval_dep__srv__Vector_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Vector_Request = {
  "ros2_rt_eval_dep::srv",
  "Vector_Request",
  _Vector_Request__cdr_serialize,
  _Vector_Request__cdr_deserialize,
  _Vector_Request__get_serialized_size,
  _Vector_Request__max_serialized_size
};

static rosidl_message_type_support_t _Vector_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Vector_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_rt_eval_dep, srv, Vector_Request)() {
  return &_Vector_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2_rt_eval_dep/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2_rt_eval_dep/srv/detail/vector__struct.h"
// already included above
// #include "ros2_rt_eval_dep/srv/detail/vector__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"  // output_vector
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"  // output_vector

// forward declare type support functions


using _Vector_Response__ros_msg_type = ros2_rt_eval_dep__srv__Vector_Response;

static bool _Vector_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Vector_Response__ros_msg_type * ros_message = static_cast<const _Vector_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: output_vector
  {
    size_t size = ros_message->output_vector.size;
    auto array_ptr = ros_message->output_vector.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: duration
  {
    cdr << ros_message->duration;
  }

  // Field name: t2
  {
    cdr << ros_message->t2;
  }

  // Field name: t3
  {
    cdr << ros_message->t3;
  }

  return true;
}

static bool _Vector_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Vector_Response__ros_msg_type * ros_message = static_cast<_Vector_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: output_vector
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->output_vector.data) {
      rosidl_runtime_c__int16__Sequence__fini(&ros_message->output_vector);
    }
    if (!rosidl_runtime_c__int16__Sequence__init(&ros_message->output_vector, size)) {
      fprintf(stderr, "failed to create array for field 'output_vector'");
      return false;
    }
    auto array_ptr = ros_message->output_vector.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: duration
  {
    cdr >> ros_message->duration;
  }

  // Field name: t2
  {
    cdr >> ros_message->t2;
  }

  // Field name: t3
  {
    cdr >> ros_message->t3;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_rt_eval_dep
size_t get_serialized_size_ros2_rt_eval_dep__srv__Vector_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Vector_Response__ros_msg_type * ros_message = static_cast<const _Vector_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name output_vector
  {
    size_t array_size = ros_message->output_vector.size;
    auto array_ptr = ros_message->output_vector.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name duration
  {
    size_t item_size = sizeof(ros_message->duration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name t2
  {
    size_t item_size = sizeof(ros_message->t2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name t3
  {
    size_t item_size = sizeof(ros_message->t3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Vector_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2_rt_eval_dep__srv__Vector_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2_rt_eval_dep
size_t max_serialized_size_ros2_rt_eval_dep__srv__Vector_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: output_vector
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: duration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: t2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: t3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2_rt_eval_dep__srv__Vector_Response;
    is_plain =
      (
      offsetof(DataType, t3) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Vector_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2_rt_eval_dep__srv__Vector_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Vector_Response = {
  "ros2_rt_eval_dep::srv",
  "Vector_Response",
  _Vector_Response__cdr_serialize,
  _Vector_Response__cdr_deserialize,
  _Vector_Response__get_serialized_size,
  _Vector_Response__max_serialized_size
};

static rosidl_message_type_support_t _Vector_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Vector_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_rt_eval_dep, srv, Vector_Response)() {
  return &_Vector_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ros2_rt_eval_dep/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2_rt_eval_dep/srv/vector.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Vector__callbacks = {
  "ros2_rt_eval_dep::srv",
  "Vector",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_rt_eval_dep, srv, Vector_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_rt_eval_dep, srv, Vector_Response)(),
};

static rosidl_service_type_support_t Vector__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Vector__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2_rt_eval_dep, srv, Vector)() {
  return &Vector__handle;
}

#if defined(__cplusplus)
}
#endif
