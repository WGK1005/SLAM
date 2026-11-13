cmake_minimum_required(VERSION 3.8)
project(wj_716_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  src
  ${Boost_INCLUDE_DIRS}
)

add_executable(wj_716_lidar_node
  src/async_client.cpp
  src/wj_716_lidar_protocol.cpp
  src/wj_716_lidar_node.cpp
)

target_link_libraries(wj_716_lidar_node
  ${Boost_LIBRARIES}
  pthread
)

ament_target_dependencies(wj_716_lidar_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  wj_716_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
