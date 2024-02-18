cmake_minimum_required(VERSION 3.8)
project(service_forward)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
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


add_executable(client_main src/client.cpp)
ament_target_dependencies(client_main ${dependencies})
target_link_libraries(client_main ${PROJECT_NAME})

add_executable(server_main src/server.cpp)
ament_target_dependencies(server_main ${dependencies})
target_link_libraries(server_main ${PROJECT_NAME})

install(TARGETScmake_minimum_required(VERSION 3.5)
project(service_forward_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Information.msg"
  "srv/GetInformation.srv"
  "action/GenerateInformation.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
  ${PROJECT_NAME}
  client_main
  server_main
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
  add_subdirectory(test)
endif()

ament_package()