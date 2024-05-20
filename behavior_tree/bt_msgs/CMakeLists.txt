cmake_minimum_required(VERSION 3.8)
project(bt_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  geometry_msgs
  builtin_interfaces
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  DEPENDENCIES ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
