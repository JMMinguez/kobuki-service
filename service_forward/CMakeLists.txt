cmake_minimum_required(VERSION 3.8)
project(service_forward)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(service_forward_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  service_forward_interfaces
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/service_forward/ClientNode.cpp
  src/service_forward/ServerNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(client src/client.cpp)
ament_target_dependencies(client ${dependencies})
target_link_libraries(client ${PROJECT_NAME})

add_executable(server src/server.cpp)
ament_target_dependencies(server ${dependencies})
target_link_libraries(server ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  client
  server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()