cmake_minimum_required(VERSION 3.8)
project(dobot_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate Messages (จุดสำคัญ!)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  DEPENDENCIES std_msgs
)

# 3. Export
ament_export_dependencies(rosidl_default_runtime)

ament_package()