cmake_minimum_required(VERSION 3.5)
project(ros2_rt_eval)


# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# add_compile_options(-std=c++17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rosidl_default_generators  REQUIRED)
find_package(ros2_rt_eval_dep REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME} #_srv
#   "srv/Vector.srv"
#   # LIBRARY_NAME ${PROJECT_NAME}
# )

add_executable(rt_srv_node src/rt_srv_node.cpp)
ament_target_dependencies(rt_srv_node rclcpp ros2_rt_eval_dep)

# add_executable(rt_cli_node src/rt_cli_node.cpp )
# ament_target_dependencies(rt_cli_node rclcpp ros2_rt_eval_dep)

# add_executable(rt_cli_node2 src/rt_cli_node2.cpp )
# ament_target_dependencies(rt_cli_node2 rclcpp ros2_rt_eval_dep)

add_executable(rt_cli_node src/rt_cli_node.cpp )
ament_target_dependencies(rt_cli_node rclcpp ros2_rt_eval_dep)

# add_executable(call_client_service src/rt_eval_node.cpp)
# ament_target_dependencies(call_client_service rclcpp ros2_rt_eval_dep)


# # Ensure the generated headers are available to your executables
# rosidl_get_typesupport_target(targets ${PROJECT_NAME} "rosidl_typesupport_cpp")

# # Link the service headers with the executables
# target_link_libraries(rt_srv_node ${targets})
# target_link_libraries(rt_cli_node ${targets})



install(TARGETS
  rt_srv_node
  rt_cli_node
  # rt_cli_node2
  # rt_cli_node3
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
