cmake_minimum_required(VERSION 3.8)
project(behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gnc_msgs REQUIRED)
find_package(gnc_core REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  nav2_costmap_2d
  tf2_ros
  geometry_msgs
  gnc_msgs
  gnc_core
  pluginlib
)

include_directories(include)

add_library(back_up_behavior SHARED plugins/back_up.cpp)
list(APPEND behaviors back_up_behavior)

add_library(wait_behavior SHARED plugins/wait.cpp)
list(APPEND behaviors wait_behavior)

foreach(behavior ${behaviors})
  ament_target_dependencies(${behavior} ${dependencies})
endforeach()

add_executable(behavior_server src/behavior_server.cpp src/main.cpp)
ament_target_dependencies(behavior_server ${dependencies})

pluginlib_export_plugin_description_file(gnc_core behavior_plugins.xml)

install(TARGETS ${behaviors}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS behavior_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES behavior_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
