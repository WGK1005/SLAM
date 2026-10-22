cmake_minimum_required(VERSION 3.16)
project(o3d_smoke LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ARM 版 Open3D
set(Open3D_DIR "$ENV{HOME}/open3d141_arm/open3d141/lib/cmake/Open3D")
find_package(Open3D REQUIRED)
message(STATUS "Open3D_VERSION = ${Open3D_VERSION}")

# 和 open3d_loc 的 CMakeLists 一样：剔掉 Open3D 配置里写死的 librt/libc++ 路径
# （Ubuntu 22.04 上这些库的位置/名字变了）
foreach(target Open3D::Open3D Open3D::3rdparty_filament Open3D::3rdparty_glfw
               Open3D::3rdparty_rply Open3D::3rdparty_glew)
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

add_executable(o3d_smoke o3d_smoke.cpp)
target_link_libraries(o3d_smoke PRIVATE Open3D::Open3D)
