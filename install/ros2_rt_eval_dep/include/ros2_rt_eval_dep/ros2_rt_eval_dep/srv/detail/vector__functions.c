// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_rt_eval_dep:srv/Vector.idl
// generated code does not contain a copyright notice
#include "ros2_rt_eval_dep/srv/detail/vector__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `input_vector`
// Member `client_id_vector`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2_rt_eval_dep__srv__Vector_Request__init(ros2_rt_eval_dep__srv__Vector_Request * msg)
{
  if (!msg) {
    return false;
  }
  // input_vector
  if (!rosidl_runtime_c__int16__Sequence__init(&msg->input_vector, 0)) {
    ros2_rt_eval_dep__srv__Vector_Request__fini(msg);
    return false;
  }
  // client_id_vector
  if (!rosidl_runtime_c__int16__Sequence__init(&msg->client_id_vector, 0)) {
    ros2_rt_eval_dep__srv__Vector_Request__fini(msg);
    return false;
  }
  return true;
}

void
ros2_rt_eval_dep__srv__Vector_Request__fini(ros2_rt_eval_dep__srv__Vector_Request * msg)
{
  if (!msg) {
    return;
  }
  // input_vector
  rosidl_runtime_c__int16__Sequence__fini(&msg->input_vector);
  // client_id_vector
  rosidl_runtime_c__int16__Sequence__fini(&msg->client_id_vector);
}

bool
ros2_rt_eval_dep__srv__Vector_Request__are_equal(const ros2_rt_eval_dep__srv__Vector_Request * lhs, const ros2_rt_eval_dep__srv__Vector_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // input_vector
  if (!rosidl_runtime_c__int16__Sequence__are_equal(
      &(lhs->input_vector), &(rhs->input_vector)))
  {
    return false;
  }
  // client_id_vector
  if (!rosidl_runtime_c__int16__Sequence__are_equal(
      &(lhs->client_id_vector), &(rhs->client_id_vector)))
  {
    return false;
  }
  return true;
}

bool
ros2_rt_eval_dep__srv__Vector_Request__copy(
  const ros2_rt_eval_dep__srv__Vector_Request * input,
  ros2_rt_eval_dep__srv__Vector_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // input_vector
  if (!rosidl_runtime_c__int16__Sequence__copy(
      &(input->input_vector), &(output->input_vector)))
  {
    return false;
  }
  // client_id_vector
  if (!rosidl_runtime_c__int16__Sequence__copy(
      &(input->client_id_vector), &(output->client_id_vector)))
  {
    return false;
  }
  return true;
}

ros2_rt_eval_dep__srv__Vector_Request *
ros2_rt_eval_dep__srv__Vector_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_rt_eval_dep__srv__Vector_Request * msg = (ros2_rt_eval_dep__srv__Vector_Request *)allocator.allocate(sizeof(ros2_rt_eval_dep__srv__Vector_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_rt_eval_dep__srv__Vector_Request));
  bool success = ros2_rt_eval_dep__srv__Vector_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_rt_eval_dep__srv__Vector_Request__destroy(ros2_rt_eval_dep__srv__Vector_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_rt_eval_dep__srv__Vector_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_rt_eval_dep__srv__Vector_Request__Sequence__init(ros2_rt_eval_dep__srv__Vector_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_rt_eval_dep__srv__Vector_Request * data = NULL;

  if (size) {
    data = (ros2_rt_eval_dep__srv__Vector_Request *)allocator.zero_allocate(size, sizeof(ros2_rt_eval_dep__srv__Vector_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_rt_eval_dep__srv__Vector_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_rt_eval_dep__srv__Vector_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_rt_eval_dep__srv__Vector_Request__Sequence__fini(ros2_rt_eval_dep__srv__Vector_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_rt_eval_dep__srv__Vector_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_rt_eval_dep__srv__Vector_Request__Sequence *
ros2_rt_eval_dep__srv__Vector_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_rt_eval_dep__srv__Vector_Request__Sequence * array = (ros2_rt_eval_dep__srv__Vector_Request__Sequence *)allocator.allocate(sizeof(ros2_rt_eval_dep__srv__Vector_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_rt_eval_dep__srv__Vector_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_rt_eval_dep__srv__Vector_Request__Sequence__destroy(ros2_rt_eval_dep__srv__Vector_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_rt_eval_dep__srv__Vector_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_rt_eval_dep__srv__Vector_Request__Sequence__are_equal(const ros2_rt_eval_dep__srv__Vector_Request__Sequence * lhs, const ros2_rt_eval_dep__srv__Vector_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_rt_eval_dep__srv__Vector_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_rt_eval_dep__srv__Vector_Request__Sequence__copy(
  const ros2_rt_eval_dep__srv__Vector_Request__Sequence * input,
  ros2_rt_eval_dep__srv__Vector_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_rt_eval_dep__srv__Vector_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_rt_eval_dep__srv__Vector_Request * data =
      (ros2_rt_eval_dep__srv__Vector_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_rt_eval_dep__srv__Vector_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_rt_eval_dep__srv__Vector_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_rt_eval_dep__srv__Vector_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `output_vector`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2_rt_eval_dep__srv__Vector_Response__init(ros2_rt_eval_dep__srv__Vector_Response * msg)
{
  if (!msg) {
    return false;
  }
  // output_vector
  if (!rosidl_runtime_c__int16__Sequence__init(&msg->output_vector, 0)) {
    ros2_rt_eval_dep__srv__Vector_Response__fini(msg);
    return false;
  }
  // duration
  // t2
  // t3
  return true;
}

void
ros2_rt_eval_dep__srv__Vector_Response__fini(ros2_rt_eval_dep__srv__Vector_Response * msg)
{
  if (!msg) {
    return;
  }
  // output_vector
  rosidl_runtime_c__int16__Sequence__fini(&msg->output_vector);
  // duration
  // t2
  // t3
}

bool
ros2_rt_eval_dep__srv__Vector_Response__are_equal(const ros2_rt_eval_dep__srv__Vector_Response * lhs, const ros2_rt_eval_dep__srv__Vector_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // output_vector
  if (!rosidl_runtime_c__int16__Sequence__are_equal(
      &(lhs->output_vector), &(rhs->output_vector)))
  {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  // t2
  if (lhs->t2 != rhs->t2) {
    return false;
  }
  // t3
  if (lhs->t3 != rhs->t3) {
    return false;
  }
  return true;
}

bool
ros2_rt_eval_dep__srv__Vector_Response__copy(
  const ros2_rt_eval_dep__srv__Vector_Response * input,
  ros2_rt_eval_dep__srv__Vector_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // output_vector
  if (!rosidl_runtime_c__int16__Sequence__copy(
      &(input->output_vector), &(output->output_vector)))
  {
    return false;
  }
  // duration
  output->duration = input->duration;
  // t2
  output->t2 = input->t2;
  // t3
  output->t3 = input->t3;
  return true;
}

ros2_rt_eval_dep__srv__Vector_Response *
ros2_rt_eval_dep__srv__Vector_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_rt_eval_dep__srv__Vector_Response * msg = (ros2_rt_eval_dep__srv__Vector_Response *)allocator.allocate(sizeof(ros2_rt_eval_dep__srv__Vector_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_rt_eval_dep__srv__Vector_Response));
  bool success = ros2_rt_eval_dep__srv__Vector_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_rt_eval_dep__srv__Vector_Response__destroy(ros2_rt_eval_dep__srv__Vector_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_rt_eval_dep__srv__Vector_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_rt_eval_dep__srv__Vector_Response__Sequence__init(ros2_rt_eval_dep__srv__Vector_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_rt_eval_dep__srv__Vector_Response * data = NULL;

  if (size) {
    data = (ros2_rt_eval_dep__srv__Vector_Response *)allocator.zero_allocate(size, sizeof(ros2_rt_eval_dep__srv__Vector_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_rt_eval_dep__srv__Vector_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_rt_eval_dep__srv__Vector_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_rt_eval_dep__srv__Vector_Response__Sequence__fini(ros2_rt_eval_dep__srv__Vector_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_rt_eval_dep__srv__Vector_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_rt_eval_dep__srv__Vector_Response__Sequence *
ros2_rt_eval_dep__srv__Vector_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_rt_eval_dep__srv__Vector_Response__Sequence * array = (ros2_rt_eval_dep__srv__Vector_Response__Sequence *)allocator.allocate(sizeof(ros2_rt_eval_dep__srv__Vector_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_rt_eval_dep__srv__Vector_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_rt_eval_dep__srv__Vector_Response__Sequence__destroy(ros2_rt_eval_dep__srv__Vector_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_rt_eval_dep__srv__Vector_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_rt_eval_dep__srv__Vector_Response__Sequence__are_equal(const ros2_rt_eval_dep__srv__Vector_Response__Sequence * lhs, const ros2_rt_eval_dep__srv__Vector_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_rt_eval_dep__srv__Vector_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_rt_eval_dep__srv__Vector_Response__Sequence__copy(
  const ros2_rt_eval_dep__srv__Vector_Response__Sequence * input,
  ros2_rt_eval_dep__srv__Vector_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_rt_eval_dep__srv__Vector_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_rt_eval_dep__srv__Vector_Response * data =
      (ros2_rt_eval_dep__srv__Vector_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_rt_eval_dep__srv__Vector_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_rt_eval_dep__srv__Vector_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_rt_eval_dep__srv__Vector_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
