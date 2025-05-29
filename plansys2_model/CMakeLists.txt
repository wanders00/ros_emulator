cmake_minimum_required(VERSION 3.5)
project(plansys2_model)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(emulation_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  plansys2_msgs
  plansys2_executor
  emulation_msgs
)

include_directories(
  include
)

add_executable(gantry_move_action_node src/gantry_move_action_node.cpp)
add_executable(gantry_calibrate_action_node src/gantry_calibrate_action_node.cpp)
add_executable(gantry_lock_action_node src/gantry_lock_action_node.cpp)
add_executable(gantry_unlock_action_node src/gantry_unlock_action_node.cpp)

add_executable(robot_move_action_node src/robot_move_action_node.cpp)
add_executable(robot_mount_action_node src/robot_mount_action_node.cpp)
add_executable(robot_unmount_action_node src/robot_unmount_action_node.cpp)

ament_target_dependencies(gantry_move_action_node ${dependencies})
ament_target_dependencies(gantry_calibrate_action_node ${dependencies})
ament_target_dependencies(gantry_lock_action_node ${dependencies})
ament_target_dependencies(gantry_unlock_action_node ${dependencies})

ament_target_dependencies(robot_move_action_node ${dependencies})
ament_target_dependencies(robot_mount_action_node ${dependencies})
ament_target_dependencies(robot_unmount_action_node ${dependencies})

install(DIRECTORY launch pddl include DESTINATION share/${PROJECT_NAME})

install(TARGETS
  gantry_move_action_node
  gantry_calibrate_action_node
  gantry_lock_action_node
  gantry_unlock_action_node
  robot_move_action_node
  robot_mount_action_node
  robot_unmount_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
