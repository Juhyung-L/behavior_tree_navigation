cmake_minimum_required(VERSION 3.8)
project(a_star_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  rclcpp_lifecycle
  nav2_util
  lifecycle_msgs
)

include_directories(include)

add_executable(astar_path_planner src/main.cpp src/a_star_node.cpp)
ament_target_dependencies(astar_path_planner ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

INSTALL(TARGETS
  astar_path_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
