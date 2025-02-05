// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_rt_eval_dep:srv/CallClientSrv.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__BUILDER_HPP_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_rt_eval_dep/srv/detail/call_client_srv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_rt_eval_dep
{

namespace srv
{

namespace builder
{

class Init_CallClientSrv_Request_num_calls
{
public:
  Init_CallClientSrv_Request_num_calls()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2_rt_eval_dep::srv::CallClientSrv_Request num_calls(::ros2_rt_eval_dep::srv::CallClientSrv_Request::_num_calls_type arg)
  {
    msg_.num_calls = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::CallClientSrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_rt_eval_dep::srv::CallClientSrv_Request>()
{
  return ros2_rt_eval_dep::srv::builder::Init_CallClientSrv_Request_num_calls();
}

}  // namespace ros2_rt_eval_dep


namespace ros2_rt_eval_dep
{

namespace srv
{

namespace builder
{

class Init_CallClientSrv_Response_latencies
{
public:
  Init_CallClientSrv_Response_latencies()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2_rt_eval_dep::srv::CallClientSrv_Response latencies(::ros2_rt_eval_dep::srv::CallClientSrv_Response::_latencies_type arg)
  {
    msg_.latencies = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_rt_eval_dep::srv::CallClientSrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_rt_eval_dep::srv::CallClientSrv_Response>()
{
  return ros2_rt_eval_dep::srv::builder::Init_CallClientSrv_Response_latencies();
}

}  // namespace ros2_rt_eval_dep

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__BUILDER_HPP_
