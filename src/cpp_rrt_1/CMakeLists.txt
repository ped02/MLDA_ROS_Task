cmake_minimum_required(VERSION 3.5)
project(cpp_rrt_1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(rosidl_default_generators REQUIRED)

add_executable(read_image_node src/cpp_read_image_node.cpp)
ament_target_dependencies(
  read_image_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
)

add_executable(map_server_node src/cpp_map_server_node.cpp)
ament_target_dependencies(
  map_server_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "cv_bridge"
)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/StartPlan.msg"
)

install(TARGETS
  read_image_node
  map_server_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
