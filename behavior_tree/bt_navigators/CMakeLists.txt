cmake_minimum_required(VERSION 3.8)
project(bt_navigators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gnc_msgs REQUIRED)
find_package(gnc_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(plugin_dependencies
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp
  nav2_util
  tf2_ros
  geometry_msgs
  gnc_msgs
  gnc_core
  nav_msgs
  ament_index_cpp
)

set(bt_engine_dependencies
  rclcpp
  behaviortree_cpp
)

include_directories(include)

add_library(bt_engine SHARED src/bt_engine.cpp)
ament_target_dependencies(bt_engine ${bt_engine_dependencies})

add_library(navigate_to_pose_navigator SHARED plugins/navigate_to_pose_navigator.cpp)
ament_target_dependencies(navigate_to_pose_navigator ${plugin_dependencies})
target_link_libraries(navigate_to_pose_navigator bt_engine)

install(TARGETS bt_engine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS navigate_to_pose_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(gnc_core navigator_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(groot2publisher_test test/groot2publisher_test.cpp)
  ament_target_dependencies(groot2publisher_test nav2_util behaviortree_cpp ament_index_cpp)
endif()

install(TARGETS groot2publisher_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(navigate_to_pose_navigator)
ament_export_dependencies(${plugin_dependencies})
ament_package()
