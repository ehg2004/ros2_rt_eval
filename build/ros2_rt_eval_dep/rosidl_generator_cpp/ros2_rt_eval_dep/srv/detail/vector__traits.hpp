// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_rt_eval_dep:srv/Vector.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__TRAITS_HPP_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_rt_eval_dep/srv/detail/vector__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_rt_eval_dep
{

namespace srv
{

inline void to_flow_style_yaml(
  const Vector_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: input_vector
  {
    if (msg.input_vector.size() == 0) {
      out << "input_vector: []";
    } else {
      out << "input_vector: [";
      size_t pending_items = msg.input_vector.size();
      for (auto item : msg.input_vector) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: client_id_vector
  {
    if (msg.client_id_vector.size() == 0) {
      out << "client_id_vector: []";
    } else {
      out << "client_id_vector: [";
      size_t pending_items = msg.client_id_vector.size();
      for (auto item : msg.client_id_vector) {
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
  const Vector_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: input_vector
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.input_vector.size() == 0) {
      out << "input_vector: []\n";
    } else {
      out << "input_vector:\n";
      for (auto item : msg.input_vector) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: client_id_vector
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.client_id_vector.size() == 0) {
      out << "client_id_vector: []\n";
    } else {
      out << "client_id_vector:\n";
      for (auto item : msg.client_id_vector) {
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

inline std::string to_yaml(const Vector_Request & msg, bool use_flow_style = false)
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
  const ros2_rt_eval_dep::srv::Vector_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_rt_eval_dep::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_rt_eval_dep::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_rt_eval_dep::srv::Vector_Request & msg)
{
  return ros2_rt_eval_dep::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_rt_eval_dep::srv::Vector_Request>()
{
  return "ros2_rt_eval_dep::srv::Vector_Request";
}

template<>
inline const char * name<ros2_rt_eval_dep::srv::Vector_Request>()
{
  return "ros2_rt_eval_dep/srv/Vector_Request";
}

template<>
struct has_fixed_size<ros2_rt_eval_dep::srv::Vector_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_rt_eval_dep::srv::Vector_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_rt_eval_dep::srv::Vector_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2_rt_eval_dep
{

namespace srv
{

inline void to_flow_style_yaml(
  const Vector_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: output_vector
  {
    if (msg.output_vector.size() == 0) {
      out << "output_vector: []";
    } else {
      out << "output_vector: [";
      size_t pending_items = msg.output_vector.size();
      for (auto item : msg.output_vector) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << ", ";
  }

  // member: t2
  {
    out << "t2: ";
    rosidl_generator_traits::value_to_yaml(msg.t2, out);
    out << ", ";
  }

  // member: t3
  {
    out << "t3: ";
    rosidl_generator_traits::value_to_yaml(msg.t3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Vector_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: output_vector
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.output_vector.size() == 0) {
      out << "output_vector: []\n";
    } else {
      out << "output_vector:\n";
      for (auto item : msg.output_vector) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }

  // member: t2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t2: ";
    rosidl_generator_traits::value_to_yaml(msg.t2, out);
    out << "\n";
  }

  // member: t3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t3: ";
    rosidl_generator_traits::value_to_yaml(msg.t3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Vector_Response & msg, bool use_flow_style = false)
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
  const ros2_rt_eval_dep::srv::Vector_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_rt_eval_dep::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_rt_eval_dep::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_rt_eval_dep::srv::Vector_Response & msg)
{
  return ros2_rt_eval_dep::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_rt_eval_dep::srv::Vector_Response>()
{
  return "ros2_rt_eval_dep::srv::Vector_Response";
}

template<>
inline const char * name<ros2_rt_eval_dep::srv::Vector_Response>()
{
  return "ros2_rt_eval_dep/srv/Vector_Response";
}

template<>
struct has_fixed_size<ros2_rt_eval_dep::srv::Vector_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_rt_eval_dep::srv::Vector_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_rt_eval_dep::srv::Vector_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_rt_eval_dep::srv::Vector>()
{
  return "ros2_rt_eval_dep::srv::Vector";
}

template<>
inline const char * name<ros2_rt_eval_dep::srv::Vector>()
{
  return "ros2_rt_eval_dep/srv/Vector";
}

template<>
struct has_fixed_size<ros2_rt_eval_dep::srv::Vector>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_rt_eval_dep::srv::Vector_Request>::value &&
    has_fixed_size<ros2_rt_eval_dep::srv::Vector_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_rt_eval_dep::srv::Vector>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_rt_eval_dep::srv::Vector_Request>::value &&
    has_bounded_size<ros2_rt_eval_dep::srv::Vector_Response>::value
  >
{
};

template<>
struct is_service<ros2_rt_eval_dep::srv::Vector>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_rt_eval_dep::srv::Vector_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_rt_eval_dep::srv::Vector_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__TRAITS_HPP_
