cmake_minimum_required(VERSION 3.5)
project(robotarm_py_cpp)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(object_srv REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(srv_files
  "srv/DetectionFromRealsense.srv")

ament_python_install_package(scripts/)

add_executable(client src/unityBluetooth.cpp)

ament_target_dependencies(client
rcl
rclcpp
"object_srv")

install(PROGRAMS
  scripts/robotArm.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  client
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(client
  bluetooth
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectionFromRealsense.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
