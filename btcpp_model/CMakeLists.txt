cmake_minimum_required(VERSION 3.8)
project(btcpp_model LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(emulation_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_executable(btcpp_model
  src/main.cpp
  src/robot_emulator_action.cpp
  src/gantry_emulator_action.cpp
)

ament_target_dependencies(btcpp_model
  rclcpp
  behaviortree_cpp
  emulation_msgs
  ament_index_cpp
)

install(TARGETS btcpp_model
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
