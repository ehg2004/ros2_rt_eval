// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_rt_eval_dep:srv/CallClientSrv.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__TRAITS_HPP_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_rt_eval_dep/srv/detail/call_client_srv__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_rt_eval_dep
{

namespace srv
{

inline void to_flow_style_yaml(
  const CallClientSrv_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: num_calls
  {
    out << "num_calls: ";
    rosidl_generator_traits::value_to_yaml(msg.num_calls, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CallClientSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num_calls
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_calls: ";
    rosidl_generator_traits::value_to_yaml(msg.num_calls, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CallClientSrv_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2_rt_eval_dep

namespace rosidl_generator_traits
{

[[deprecated("use ros2_rt_eval_dep::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_rt_eval_dep::srv::CallClientSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_rt_eval_dep::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_rt_eval_dep::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_rt_eval_dep::srv::CallClientSrv_Request & msg)
{
  return ros2_rt_eval_dep::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_rt_eval_dep::srv::CallClientSrv_Request>()
{
  return "ros2_rt_eval_dep::srv::CallClientSrv_Request";
}

template<>
inline const char * name<ros2_rt_eval_dep::srv::CallClientSrv_Request>()
{
  return "ros2_rt_eval_dep/srv/CallClientSrv_Request";
}

template<>
struct has_fixed_size<ros2_rt_eval_dep::srv::CallClientSrv_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_rt_eval_dep::srv::CallClientSrv_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_rt_eval_dep::srv::CallClientSrv_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2_rt_eval_dep
{

namespace srv
{

inline void to_flow_style_yaml(
  const CallClientSrv_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: latencies
  {
    if (msg.latencies.size() == 0) {
      out << "latencies: []";
    } else {
      out << "latencies: [";
      size_t pending_items = msg.latencies.size();
      for (auto item : msg.latencies) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CallClientSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latencies
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.latencies.size() == 0) {
      out << "latencies: []\n";
    } else {
      out << "latencies:\n";
      for (auto item : msg.latencies) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CallClientSrv_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2_rt_eval_dep

namespace rosidl_generator_traits
{

[[deprecated("use ros2_rt_eval_dep::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_rt_eval_dep::srv::CallClientSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_rt_eval_dep::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_rt_eval_dep::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_rt_eval_dep::srv::CallClientSrv_Response & msg)
{
  return ros2_rt_eval_dep::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_rt_eval_dep::srv::CallClientSrv_Response>()
{
  return "ros2_rt_eval_dep::srv::CallClientSrv_Response";
}

template<>
inline const char * name<ros2_rt_eval_dep::srv::CallClientSrv_Response>()
{
  return "ros2_rt_eval_dep/srv/CallClientSrv_Response";
}

template<>
struct has_fixed_size<ros2_rt_eval_dep::srv::CallClientSrv_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_rt_eval_dep::srv::CallClientSrv_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_rt_eval_dep::srv::CallClientSrv_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_rt_eval_dep::srv::CallClientSrv>()
{
  return "ros2_rt_eval_dep::srv::CallClientSrv";
}

template<>
inline const char * name<ros2_rt_eval_dep::srv::CallClientSrv>()
{
  return "ros2_rt_eval_dep/srv/CallClientSrv";
}

template<>
struct has_fixed_size<ros2_rt_eval_dep::srv::CallClientSrv>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_rt_eval_dep::srv::CallClientSrv_Request>::value &&
    has_fixed_size<ros2_rt_eval_dep::srv::CallClientSrv_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_rt_eval_dep::srv::CallClientSrv>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_rt_eval_dep::srv::CallClientSrv_Request>::value &&
    has_bounded_size<ros2_rt_eval_dep::srv::CallClientSrv_Response>::value
  >
{
};

template<>
struct is_service<ros2_rt_eval_dep::srv::CallClientSrv>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_rt_eval_dep::srv::CallClientSrv_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_rt_eval_dep::srv::CallClientSrv_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__TRAITS_HPP_
