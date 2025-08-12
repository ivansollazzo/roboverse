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
find_package(rosidl_default_generators REQUIRED)

## INCLUDE HEADERS DIRECTORY
include_directories(include)

## ADD EXECUTABLES
# Kinematics Controller Executable
add_executable(kinematics_controller src/KinematicsController.cpp)

# Task Manager Executable
add_executable(task_manager src/TaskManager.cpp)

# Rendez-vous Manager Executable
add_executable(rendezvous_manager src/RendezvousManager.cpp)

# Knowledge Register Executable
add_executable(knowledge_register src/KnowledgeRegister.cpp)

# Sensors Manager Executable
add_executable(sensors_manager src/SensorsManager.cpp)

## ROSIDL INTERFACES GENERATION
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UpdateKnowledge.srv"
  DEPENDENCIES builtin_interfaces
)

## ROSIDL TARGET INTERFACES
rosidl_target_interfaces(knowledge_register
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(task_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

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

# Rendez-vous Manager Target Dependencies
ament_target_dependencies (
  rendezvous_manager
  rclcpp
  geometry_msgs
  std_msgs
)

# Knowledge Register Target Dependencies
ament_target_dependencies(knowledge_register
  rclcpp
  ament_index_cpp
)

# Sensors Manager Target Dependencies
ament_target_dependencies(
  sensors_manager
  rclcpp
  std_msgs
  ament_index_cpp
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

# Rendez-vous Manager Install Targets
install(TARGETS rendezvous_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Knowledge Register Install Targets
install(TARGETS knowledge_register
  DESTINATION lib/${PROJECT_NAME}
)

# Sensors Manager Install Targets
install(TARGETS sensors_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
