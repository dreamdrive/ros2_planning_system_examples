cmake_minimum_required(VERSION 3.5)
project(my_simple_example)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
    moveit_ros_planning_interface
    control_msgs
    )

add_executable(move_action_node src/move_action_node_follow_joint_trajectory.cpp)
# target_include_directories(move_action_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

ament_target_dependencies(move_action_node ${dependencies})

add_executable(zero_move_action_node src/zero_move_action_node.cpp)
ament_target_dependencies(zero_move_action_node ${dependencies})

add_executable(joint_move_action_node src/joint_move_action_node.cpp)
ament_target_dependencies(joint_move_action_node ${dependencies})

# add_executable(charge_action_node src/charge_action_node.cpp)
# ament_target_dependencies(charge_action_node ${dependencies})

# add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
# ament_target_dependencies(ask_charge_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  zero_move_action_node
  joint_move_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# charge_action_node
# ask_charge_action_node

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()