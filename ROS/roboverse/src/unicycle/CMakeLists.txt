cmake_minimum_required(VERSION 3.8)
project(unicycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## INCLUDE HEADERS DIRECTORY
include_directories(include)

## ADD EXECUTABLES
# Kinematics Controller Executable
add_executable(kinematics_controller src/KinematicsController.cpp)

# Task Manager Executable
add_executable(task_manager src/TaskManager.cpp)

## AMENT TARGET DEPENDENCIES
# Kinematics Controller Target Dependencies
ament_target_dependencies(
  kinematics_controller
  rclcpp
  geometry_msgs
)

# Task Manager Target Dependencies
ament_target_dependencies(
  task_manager
  rclcpp
  geometry_msgs
  std_msgs
)

## INSTALL TARGETS
# Kinematics Controller Install Targets
install(TARGETS kinematics_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Task Manager Install Targets
install(TARGETS task_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
