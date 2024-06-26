cmake_minimum_required(VERSION 3.8)
project(bt_navigator_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gnc_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  tf2_ros
  nav2_util
  pluginlib
  gnc_core
)

include_directories(include)

add_executable(bt_navigator_server src/main.cpp src/bt_navigator_server.cpp)
ament_target_dependencies(bt_navigator_server ${dependencies})

install(TARGETS
  bt_navigator_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
