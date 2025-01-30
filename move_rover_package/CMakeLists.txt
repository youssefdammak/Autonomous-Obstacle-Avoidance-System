cmake_minimum_required(VERSION 3.8)
project(move_rover_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(move_rover_action REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_executable src/cpp_client.cpp)

# Link dependencies
ament_target_dependencies(cpp_executable rclcpp rclcpp_action move_rover_action)

# Install Cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Python install
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/py_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
