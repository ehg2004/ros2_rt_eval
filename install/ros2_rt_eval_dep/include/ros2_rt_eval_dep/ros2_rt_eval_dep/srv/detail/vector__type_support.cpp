// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros2_rt_eval_dep:srv/Vector.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2_rt_eval_dep/srv/detail/vector__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_rt_eval_dep
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void Vector_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_rt_eval_dep::srv::Vector_Request(_init);
}

void Vector_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_rt_eval_dep::srv::Vector_Request *>(message_memory);
  typed_message->~Vector_Request();
}

size_t size_function__Vector_Request__input_vector(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Vector_Request__input_vector(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Vector_Request__input_vector(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Vector_Request__input_vector(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__Vector_Request__input_vector(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__Vector_Request__input_vector(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__Vector_Request__input_vector(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

void resize_function__Vector_Request__input_vector(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Vector_Request__client_id_vector(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Vector_Request__client_id_vector(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Vector_Request__client_id_vector(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Vector_Request__client_id_vector(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__Vector_Request__client_id_vector(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__Vector_Request__client_id_vector(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__Vector_Request__client_id_vector(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

void resize_function__Vector_Request__client_id_vector(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Vector_Request_message_member_array[2] = {
  {
    "input_vector",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_rt_eval_dep::srv::Vector_Request, input_vector),  // bytes offset in struct
    nullptr,  // default value
    size_function__Vector_Request__input_vector,  // size() function pointer
    get_const_function__Vector_Request__input_vector,  // get_const(index) function pointer
    get_function__Vector_Request__input_vector,  // get(index) function pointer
    fetch_function__Vector_Request__input_vector,  // fetch(index, &value) function pointer
    assign_function__Vector_Request__input_vector,  // assign(index, value) function pointer
    resize_function__Vector_Request__input_vector  // resize(index) function pointer
  },
  {
    "client_id_vector",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_rt_eval_dep::srv::Vector_Request, client_id_vector),  // bytes offset in struct
    nullptr,  // default value
    size_function__Vector_Request__client_id_vector,  // size() function pointer
    get_const_function__Vector_Request__client_id_vector,  // get_const(index) function pointer
    get_function__Vector_Request__client_id_vector,  // get(index) function pointer
    fetch_function__Vector_Request__client_id_vector,  // fetch(index, &value) function pointer
    assign_function__Vector_Request__client_id_vector,  // assign(index, value) function pointer
    resize_function__Vector_Request__client_id_vector  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Vector_Request_message_members = {
  "ros2_rt_eval_dep::srv",  // message namespace
  "Vector_Request",  // message name
  2,  // number of fields
  sizeof(ros2_rt_eval_dep::srv::Vector_Request),
  Vector_Request_message_member_array,  // message members
  Vector_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Vector_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Vector_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Vector_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros2_rt_eval_dep


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_rt_eval_dep::srv::Vector_Request>()
{
  return &::ros2_rt_eval_dep::srv::rosidl_typesupport_introspection_cpp::Vector_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_rt_eval_dep, srv, Vector_Request)() {
  return &::ros2_rt_eval_dep::srv::rosidl_typesupport_introspection_cpp::Vector_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ros2_rt_eval_dep/srv/detail/vector__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_rt_eval_dep
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void Vector_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_rt_eval_dep::srv::Vector_Response(_init);
}

void Vector_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_rt_eval_dep::srv::Vector_Response *>(message_memory);
  typed_message->~Vector_Response();
}

size_t size_function__Vector_Response__output_vector(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Vector_Response__output_vector(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Vector_Response__output_vector(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Vector_Response__output_vector(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__Vector_Response__output_vector(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__Vector_Response__output_vector(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__Vector_Response__output_vector(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

void resize_function__Vector_Response__output_vector(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int16_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Vector_Response_message_member_array[4] = {
  {
    "output_vector",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_rt_eval_dep::srv::Vector_Response, output_vector),  // bytes offset in struct
    nullptr,  // default value
    size_function__Vector_Response__output_vector,  // size() function pointer
    get_const_function__Vector_Response__output_vector,  // get_const(index) function pointer
    get_function__Vector_Response__output_vector,  // get(index) function pointer
    fetch_function__Vector_Response__output_vector,  // fetch(index, &value) function pointer
    assign_function__Vector_Response__output_vector,  // assign(index, value) function pointer
    resize_function__Vector_Response__output_vector  // resize(index) function pointer
  },
  {
    "duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_rt_eval_dep::srv::Vector_Response, duration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_rt_eval_dep::srv::Vector_Response, t2),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t3",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_rt_eval_dep::srv::Vector_Response, t3),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Vector_Response_message_members = {
  "ros2_rt_eval_dep::srv",  // message namespace
  "Vector_Response",  // message name
  4,  // number of fields
  sizeof(ros2_rt_eval_dep::srv::Vector_Response),
  Vector_Response_message_member_array,  // message members
  Vector_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Vector_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Vector_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Vector_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros2_rt_eval_dep


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_rt_eval_dep::srv::Vector_Response>()
{
  return &::ros2_rt_eval_dep::srv::rosidl_typesupport_introspection_cpp::Vector_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_rt_eval_dep, srv, Vector_Response)() {
  return &::ros2_rt_eval_dep::srv::rosidl_typesupport_introspection_cpp::Vector_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "ros2_rt_eval_dep/srv/detail/vector__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace ros2_rt_eval_dep
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers Vector_service_members = {
  "ros2_rt_eval_dep::srv",  // service namespace
  "Vector",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<ros2_rt_eval_dep::srv::Vector>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t Vector_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Vector_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace ros2_rt_eval_dep


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<ros2_rt_eval_dep::srv::Vector>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::ros2_rt_eval_dep::srv::rosidl_typesupport_introspection_cpp::Vector_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros2_rt_eval_dep::srv::Vector_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::ros2_rt_eval_dep::srv::Vector_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_rt_eval_dep, srv, Vector)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<ros2_rt_eval_dep::srv::Vector>();
}

#ifdef __cplusplus
}
#endif
