cmake_minimum_required(VERSION 3.8)
project(bt_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_util REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp
  nav2_util
)

set(library_name ${PROJECT_NAME})

include_directories(include)

add_library(${library_name} SHARED src/bt_engine.cpp)
ament_target_dependencies(${library_name} ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
