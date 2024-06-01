cmake_minimum_required(VERSION 3.8)
project(a_star_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(gnc_core REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  gnc_core
)

include_directories(include)

add_library(a_star_global_planner SHARED src/a_star_global_planner.cpp)
ament_target_dependencies(a_star_global_planner ${dependencies})

install(TARGETS a_star_global_planner
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
endif()

ament_export_include_directories(include)
ament_export_libraries(a_star_global_planner)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(gnc_core planner_plugin.xml)

ament_package()
