cmake_minimum_required(VERSION 3.8)
project(controller_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gnc_core REQUIRED)
find_package(gnc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  nav2_util
  nav2_costmap_2d
  tf2_ros
  gnc_core
  gnc_msgs
  geometry_msgs
  angles
  rclcpp_lifecycle
  tf2
  rclcpp
  tf2_geometry_msgs
)

include_directories(include)

add_executable(controller_server
  src/main.cpp
  src/controller_server.cpp
  src/goal_checker.cpp
  src/progress_checker.cpp  
)
ament_target_dependencies(controller_server ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(goal_checker_test 
    test/goal_checker_test.cpp
    src/goal_checker.cpp
  )
  ament_target_dependencies(goal_checker_test ${dependencies})
  ament_add_gtest(progress_checker_test
    test/progress_checker_test.cpp
    src/progress_checker.cpp  
  )
  ament_target_dependencies(progress_checker_test ${dependencies})
endif()

install(TARGETS 
  controller_server
  goal_checker_test
  progress_checker_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
