cmake_minimum_required(VERSION 3.8)
project(dwa_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  nav_2d_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/conversions.cpp src/transform_utils.cpp)
ament_target_dependencies(${PROJECT_NAME}  ${dependencies})

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

INSTALL(DIRECTORY
  include/
  DESTINATION include/
)

INSTALL(TARGETS ${PROJECT_NAME} 
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} )
ament_export_dependencies(${dependencies})

ament_package()
