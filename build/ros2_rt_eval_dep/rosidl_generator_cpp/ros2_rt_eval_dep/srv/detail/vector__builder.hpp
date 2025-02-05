// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_rt_eval_dep:srv/Vector.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__BUILDER_HPP_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_rt_eval_dep/srv/detail/vector__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_rt_eval_dep
{

namespace srv
{

namespace builder
{

class Init_Vector_Request_client_id_vector
{
public:
  explicit Init_Vector_Request_client_id_vector(::ros2_rt_eval_dep::srv::Vector_Request & msg)
  : msg_(msg)
  {}
  ::ros2_rt_eval_dep::srv::Vector_Request client_id_vector(::ros2_rt_eval_dep::srv::Vector_Request::_client_id_vector_type arg)
  {
    msg_.client_id_vector = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::Vector_Request msg_;
};

class Init_Vector_Request_input_vector
{
public:
  Init_Vector_Request_input_vector()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vector_Request_client_id_vector input_vector(::ros2_rt_eval_dep::srv::Vector_Request::_input_vector_type arg)
  {
    msg_.input_vector = std::move(arg);
    return Init_Vector_Request_client_id_vector(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::Vector_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_rt_eval_dep::srv::Vector_Request>()
{
  return ros2_rt_eval_dep::srv::builder::Init_Vector_Request_input_vector();
}

}  // namespace ros2_rt_eval_dep


namespace ros2_rt_eval_dep
{

namespace srv
{

namespace builder
{

class Init_Vector_Response_t3
{
public:
  explicit Init_Vector_Response_t3(::ros2_rt_eval_dep::srv::Vector_Response & msg)
  : msg_(msg)
  {}
  ::ros2_rt_eval_dep::srv::Vector_Response t3(::ros2_rt_eval_dep::srv::Vector_Response::_t3_type arg)
  {
    msg_.t3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::Vector_Response msg_;
};

class Init_Vector_Response_t2
{
public:
  explicit Init_Vector_Response_t2(::ros2_rt_eval_dep::srv::Vector_Response & msg)
  : msg_(msg)
  {}
  Init_Vector_Response_t3 t2(::ros2_rt_eval_dep::srv::Vector_Response::_t2_type arg)
  {
    msg_.t2 = std::move(arg);
    return Init_Vector_Response_t3(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::Vector_Response msg_;
};

class Init_Vector_Response_duration
{
public:
  explicit Init_Vector_Response_duration(::ros2_rt_eval_dep::srv::Vector_Response & msg)
  : msg_(msg)
  {}
  Init_Vector_Response_t2 duration(::ros2_rt_eval_dep::srv::Vector_Response::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_Vector_Response_t2(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::Vector_Response msg_;
};

class Init_Vector_Response_output_vector
{
public:
  Init_Vector_Response_output_vector()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vector_Response_duration output_vector(::ros2_rt_eval_dep::srv::Vector_Response::_output_vector_type arg)
  {
    msg_.output_vector = std::move(arg);
    return Init_Vector_Response_duration(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::Vector_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_rt_eval_dep::srv::Vector_Response>()
{
  return ros2_rt_eval_dep::srv::builder::Init_Vector_Response_output_vector();
}

}  // namespace ros2_rt_eval_dep

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__BUILDER_HPP_
