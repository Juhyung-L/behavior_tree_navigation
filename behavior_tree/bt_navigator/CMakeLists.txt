cmake_minimum_required(VERSION 3.8)
project(bt_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(bt_interface REQUIRED)
find_package(gnc_msgs REQUIRED)
find_package(gnc_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  tf2_ros
  pluginlib
  nav_msgs
  geometry_msgs
  ament_index_cpp
  nav2_util
  bt_interface
  gnc_msgs
  gnc_core
)

include_directories(include)

add_executable(bt_navigator_server src/main.cpp src/bt_navigator_server.cpp)
ament_target_dependencies(bt_navigator_server ${dependencies})

add_library(navigate_to_pose_navigator SHARED src/navigate_to_pose_navigator.cpp)
ament_target_dependencies(navigate_to_pose_navigator ${dependencies})

pluginlib_export_plugin_description_file(gnc_core navigator_plugins.xml)

install(TARGETS
  navigate_to_pose_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  bt_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(navigate_to_pose_navigator)
ament_export_dependencies(${dependencies})
ament_package()
