cmake_minimum_required(VERSION 3.8)
project(ar4_joint_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(arm_controller src/arm_controller.cpp)
ament_target_dependencies(arm_controller rclcpp sensor_msgs)

install(TARGETS arm_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
