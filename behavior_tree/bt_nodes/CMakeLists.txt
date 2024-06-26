cmake_minimum_required(VERSION 3.8)
project(bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(gnc_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_util
  gnc_msgs
  nav_msgs
)

include_directories(include)

add_library(follow_path_action_bt_node SHARED plugins/action/follow_path_action.cpp)
list(APPEND bt_nodes follow_path_action_bt_node)

add_library(compute_path_to_pose_action_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
list(APPEND bt_nodes compute_path_to_pose_action_bt_node)

add_library(pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND bt_nodes pipeline_sequence_bt_node)

add_library(recovery_bt_node SHARED plugins/control/recovery.cpp)
list(APPEND bt_nodes recovery_bt_node)

add_library(round_robin_bt_node SHARED plugins/control/round_robin.cpp)
list(APPEND bt_nodes round_robin_bt_node)

add_library(back_up_action_bt_node SHARED plugins/action/back_up_action.cpp)
list(APPEND bt_nodes back_up_action_bt_node)

add_library(wait_action_bt_node SHARED plugins/action/wait_action.cpp)
list(APPEND bt_nodes wait_action_bt_node)

add_library(clear_costmap_service_bt_node SHARED plugins/action/clear_costmap_service.cpp)
list(APPEND bt_nodes clear_costmap_service_bt_node)

add_library(goal_updated_condition_bt_node SHARED plugins/condition/goal_updated_condition.cpp)
list(APPEND bt_nodes goal_updated_condition_bt_node)

add_library(rate_controller_bt_node SHARED plugins/decorator/rate_controller.cpp)
list(APPEND bt_nodes rate_controller_bt_node)

foreach(bt_node ${bt_nodes})
  ament_target_dependencies(${bt_node} ${dependencies})
  target_compile_definitions(${bt_node} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# auto generate tree nodes xml file
set(OUTPUT_XML_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bt_nodes.xml")
add_executable(generate_tree_nodes_model_xml plugins/generate_tree_nodes_model_xml.cpp)
ament_target_dependencies(generate_tree_nodes_model_xml ${dependencies})
target_link_libraries(generate_tree_nodes_model_xml ${bt_nodes})
add_custom_command(
  TARGET generate_tree_nodes_model_xml
  COMMAND generate_tree_nodes_model_xml "${OUTPUT_XML_PATH}"
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

install(TARGETS generate_tree_nodes_model_xml
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${bt_nodes}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES bt_nodes.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
  endif()

ament_export_libraries(${bt_nodes})
ament_export_dependencies(${dependencies})
ament_package()
