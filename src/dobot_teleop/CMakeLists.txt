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

# 2. Generate Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  DEPENDENCIES std_msgs
)

# 3. Export
ament_export_dependencies(rosidl_default_runtime)

# --- [แก้ตรงนี้ครับ] ---
# เปลี่ยนจาก COPY Directory เป็น Install PROGRAMS (ระบุชื่อไฟล์ชัดๆ)
# เพื่อบังคับให้มันเป็น Executable (สีเขียว) แน่นอน 100%
install(PROGRAMS
  scripts/dobot_driver.py
  scripts/mg400_interface.py
  scripts/tcp_receiver.py
  DESTINATION lib/${PROJECT_NAME}
)
# --------------------

ament_package()