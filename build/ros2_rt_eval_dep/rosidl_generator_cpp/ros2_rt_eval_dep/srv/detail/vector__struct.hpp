// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_rt_eval_dep:srv/Vector.idl
// generated code does not contain a copyright notice

#ifndef ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__STRUCT_HPP_
#define ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_rt_eval_dep__srv__Vector_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_rt_eval_dep__srv__Vector_Request __declspec(deprecated)
#endif

namespace ros2_rt_eval_dep
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Vector_Request_
{
  using Type = Vector_Request_<ContainerAllocator>;

  explicit Vector_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Vector_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _input_vector_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _input_vector_type input_vector;
  using _client_id_vector_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _client_id_vector_type client_id_vector;

  // setters for named parameter idiom
  Type & set__input_vector(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->input_vector = _arg;
    return *this;
  }
  Type & set__client_id_vector(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->client_id_vector = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_rt_eval_dep__srv__Vector_Request
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_rt_eval_dep__srv__Vector_Request
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Vector_Request_ & other) const
  {
    if (this->input_vector != other.input_vector) {
      return false;
    }
    if (this->client_id_vector != other.client_id_vector) {
      return false;
    }
    return true;
  }
  bool operator!=(const Vector_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Vector_Request_

// alias to use template instance with default allocator
using Vector_Request =
  ros2_rt_eval_dep::srv::Vector_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_rt_eval_dep


#ifndef _WIN32
# define DEPRECATED__ros2_rt_eval_dep__srv__Vector_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_rt_eval_dep__srv__Vector_Response __declspec(deprecated)
#endif

namespace ros2_rt_eval_dep
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Vector_Response_
{
  using Type = Vector_Response_<ContainerAllocator>;

  explicit Vector_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duration = 0ll;
      this->t2 = 0ll;
      this->t3 = 0ll;
    }
  }

  explicit Vector_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duration = 0ll;
      this->t2 = 0ll;
      this->t3 = 0ll;
    }
  }

  // field types and members
  using _output_vector_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _output_vector_type output_vector;
  using _duration_type =
    int64_t;
  _duration_type duration;
  using _t2_type =
    int64_t;
  _t2_type t2;
  using _t3_type =
    int64_t;
  _t3_type t3;

  // setters for named parameter idiom
  Type & set__output_vector(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->output_vector = _arg;
    return *this;
  }
  Type & set__duration(
    const int64_t & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__t2(
    const int64_t & _arg)
  {
    this->t2 = _arg;
    return *this;
  }
  Type & set__t3(
    const int64_t & _arg)
  {
    this->t3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_rt_eval_dep__srv__Vector_Response
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_rt_eval_dep__srv__Vector_Response
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Vector_Response_ & other) const
  {
    if (this->output_vector != other.output_vector) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->t2 != other.t2) {
      return false;
    }
    if (this->t3 != other.t3) {
      return false;
    }
    return true;
  }
  bool operator!=(const Vector_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Vector_Response_

// alias to use template instance with default allocator
using Vector_Response =
  ros2_rt_eval_dep::srv::Vector_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_rt_eval_dep

namespace ros2_rt_eval_dep
{

namespace srv
{

struct Vector
{
  using Request = ros2_rt_eval_dep::srv::Vector_Request;
  using Response = ros2_rt_eval_dep::srv::Vector_Response;
};

}  // namespace srv

}  // namespace ros2_rt_eval_dep

#endif  // ROS2_RT_EVAL_DEP__SRV__DETAIL__VECTOR__STRUCT_HPP_
