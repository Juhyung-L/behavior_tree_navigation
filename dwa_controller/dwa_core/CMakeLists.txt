cmake_minimum_required(VERSION 3.8)
project(dwa_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(dwa_util REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(dwa_critics REQUIRED)
find_package(gnc_core REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  nav_2d_msgs
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  dwa_util
  nav2_util
  rclcpp_lifecycle
  lifecycle_msgs
  dwa_critics
  gnc_core
)

add_library(dwa_local_planner SHARED src/dwa_local_planner.cpp)
ament_target_dependencies(dwa_local_planner ${dependencies})

install(TARGETS dwa_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(xytheta_iterator_test test/xytheta_iterator_test.cpp)
  ament_target_dependencies(xytheta_iterator_test ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(dwa_local_planner)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(gnc_core controller_plugin.xml)

ament_package()
