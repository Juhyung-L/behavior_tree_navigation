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

foreach(bt_node ${bt_nodes})
  ament_target_dependencies(${bt_node} ${dependencies})
  target_compile_definitions(${bt_node} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

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
