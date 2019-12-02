cmake_minimum_required(VERSION 3.5)
project(robotarm_control_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-lwiringPi)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(color_srv REQUIRED)

set(srv_files
  "srv/DetectionFromRealsense.srv")

add_executable(client src/unityBluetooth.cpp)
ament_target_dependencies(client
rclcpp example_interfaces
"color_srv")

add_executable(server src/robotArm.cpp)
ament_target_dependencies(server
rclcpp example_interfaces
"color_srv"
)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(server
wiringPi
crypt
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectionFromRealsense.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
