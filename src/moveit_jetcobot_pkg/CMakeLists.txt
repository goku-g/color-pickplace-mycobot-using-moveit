cmake_minimum_required(VERSION 3.22)
project(moveit_jetcobot_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_to_target_pose src/move_to_target_pose.cpp)
ament_target_dependencies(move_to_target_pose
  rclcpp
  moveit_ros_planning_interface)

install(TARGETS
  move_to_target_pose
  DESTINATION lib/${PROJECT_NAME})

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
