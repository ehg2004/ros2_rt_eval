// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_rt_eval_dep:srv/CallClientSrv.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__STRUCT_HPP_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Request __declspec(deprecated)
#endif

namespace ros2_rt_eval_dep
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CallClientSrv_Request_
{
  using Type = CallClientSrv_Request_<ContainerAllocator>;

  explicit CallClientSrv_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_calls = 0l;
    }
  }

  explicit CallClientSrv_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_calls = 0l;
    }
  }

  // field types and members
  using _num_calls_type =
    int32_t;
  _num_calls_type num_calls;

  // setters for named parameter idiom
  Type & set__num_calls(
    const int32_t & _arg)
  {
    this->num_calls = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Request
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Request
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CallClientSrv_Request_ & other) const
  {
    if (this->num_calls != other.num_calls) {
      return false;
    }
    return true;
  }
  bool operator!=(const CallClientSrv_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CallClientSrv_Request_

// alias to use template instance with default allocator
using CallClientSrv_Request =
  ros2_rt_eval_dep::srv::CallClientSrv_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_rt_eval_dep


#ifndef _WIN32
# define DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Response __declspec(deprecated)
#endif

namespace ros2_rt_eval_dep
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CallClientSrv_Response_
{
  using Type = CallClientSrv_Response_<ContainerAllocator>;

  explicit CallClientSrv_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit CallClientSrv_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _latencies_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _latencies_type latencies;

  // setters for named parameter idiom
  Type & set__latencies(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->latencies = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Response
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_rt_eval_dep__srv__CallClientSrv_Response
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CallClientSrv_Response_ & other) const
  {
    if (this->latencies != other.latencies) {
      return false;
    }
    return true;
  }
  bool operator!=(const CallClientSrv_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CallClientSrv_Response_

// alias to use template instance with default allocator
using CallClientSrv_Response =
  ros2_rt_eval_dep::srv::CallClientSrv_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_rt_eval_dep

namespace ros2_rt_eval_dep
{

namespace srv
{

struct CallClientSrv
{
  using Request = ros2_rt_eval_dep::srv::CallClientSrv_Request;
  using Response = ros2_rt_eval_dep::srv::CallClientSrv_Response;
};

}  // namespace srv

}  // namespace ros2_rt_eval_dep

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__CALL_CLIENT_SRV__STRUCT_HPP_
