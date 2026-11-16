cmake_minimum_required(VERSION 3.8)
project(rolling_map_updater)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Auto-detect Open3D (matching workspace convention)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  if(EXISTS "/home/orangepi/open3d141_arm/open3d141/lib/cmake/Open3D")
    set(Open3D_DIR "/home/orangepi/open3d141_arm/open3d141/lib/cmake/Open3D")
  elseif(EXISTS "$ENV{HOME}/open3d141_arm/open3d141/lib/cmake/Open3D")
    set(Open3D_DIR "$ENV{HOME}/open3d141_arm/open3d141/lib/cmake/Open3D")
  endif()
else()
  if(EXISTS "$ENV{HOME}/thirdparty/open3d141/open3d141/lib/cmake/Open3D")
    set(Open3D_DIR "$ENV{HOME}/thirdparty/open3d141/open3d141/lib/cmake/Open3D")
  elseif(EXISTS "/home/wgk/open3d141/open3d141/lib/cmake/Open3D")
    set(Open3D_DIR "/home/wgk/open3d141/open3d141/lib/cmake/Open3D")
  endif()
endif()
find_package(Open3D REQUIRED)

foreach(target Open3D::Open3D Open3D::3rdparty_filament Open3D::3rdparty_glfw Open3D::3rdparty_rply Open3D::3rdparty_glew)
  if(TARGET ${target})
    get_target_property(LINK_LIBS ${target} INTERFACE_LINK_LIBRARIES)
    if(LINK_LIBS)
      set(FILTERED_LIBS "")
      foreach(lib ${LINK_LIBS})
        if(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
           NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
           NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
          list(APPEND FILTERED_LIBS ${lib})
        endif()
      endforeach()
      set_target_properties(${target} PROPERTIES INTERFACE_LINK_LIBRARIES "${FILTERED_LIBS}")
    endif()
  endif()
endforeach()

if(Open3D_LIBRARIES)
  set(FILTERED_O3D_LIBS "")
  foreach(lib ${Open3D_LIBRARIES})
    if(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
       NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
       NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
      list(APPEND FILTERED_O3D_LIBS ${lib})
    endif()
  endforeach()
  set(Open3D_LIBRARIES ${FILTERED_O3D_LIBS})
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)

add_library(rolling_voxel_map
  src/rolling_voxel_map.cpp
)
target_link_libraries(rolling_voxel_map
  ${Open3D_LIBRARIES}
)

add_executable(rolling_map_node
  src/rolling_map_node.cpp
)
target_link_libraries(rolling_map_node
  rolling_voxel_map
  ${Open3D_LIBRARIES}
)
ament_target_dependencies(rolling_map_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

add_executable(test_rolling_keep_new_delete_old
  test/test_rolling_keep_new_delete_old.cpp
)
target_link_libraries(test_rolling_keep_new_delete_old
  rolling_voxel_map
  ${Open3D_LIBRARIES}
)

install(TARGETS
  rolling_voxel_map
  rolling_map_node
  test_rolling_keep_new_delete_old
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
