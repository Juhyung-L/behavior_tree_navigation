cmake_minimum_required(VERSION 3.8)
project(planner_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gnc_core REQUIRED)
find_package(gnc_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  pluginlib
  gnc_core
  gnc_msgs
)

include_directories(include)

add_executable(planner_server src/main.cpp src/planner_server.cpp)
ament_target_dependencies(planner_server ${dependencies})

install(TARGETS planner_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
