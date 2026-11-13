cmake_minimum_required(VERSION 3.14)
project(scu_tdt_v1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-deprecated-enum-enum-conversion")

find_package(Eigen3 REQUIRED)
find_package(OpenCV QUIET)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# 1. 核心规划算法静态库 (YAstar + KinodynamicAstar + MinimumSnap + SquareSFC + SCURM Recovery)
add_library(scu_tdt_planner STATIC
    src/YAstar/yastar.cpp
    src/YAstar/kinodynamicAstar.cpp
    src/MinimumSnapOsqp/minimumSnap.cpp
    src/MinimumSnapOsqp/sfcSquare.cpp
)
target_link_libraries(scu_tdt_planner Eigen3::Eigen ${OpenCV_LIBS})

# 2. 独立实机地图离线测试与出图程序
add_executable(test_lab_nav
    tools/test_lab_nav.cpp
)
target_link_libraries(test_lab_nav scu_tdt_planner Eigen3::Eigen ${OpenCV_LIBS})

# 2b. ROBOCON 2027 宇树 Go2 障碍赛越障与梅花桩落足点规划程序
add_executable(plan_rc_obstacles
    tools/plan_rc_obstacles.cpp
)
target_link_libraries(plan_rc_obstacles scu_tdt_planner Eigen3::Eigen ${OpenCV_LIBS})

# 3. TDT-Nav-Kit 原版 RMUC 2024/2025 赛场演示程序 (play)
if(OpenCV_FOUND)
    add_executable(play src/orig_demo_main.cpp)
    target_link_libraries(play scu_tdt_planner Eigen3::Eigen ${OpenCV_LIBS})
endif()

# 4. 如果当前环境加载了 ROS 2 Humble，自动编译 ROS 2 实时规划控制节点 scu_tdt_nav_node
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    find_package(rclcpp REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)

    add_executable(scu_tdt_nav_node src/scu_tdt_nav_node.cpp)
    ament_target_dependencies(scu_tdt_nav_node
        rclcpp
        nav_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
    )
    target_link_libraries(scu_tdt_nav_node scu_tdt_planner Eigen3::Eigen)

    install(TARGETS scu_tdt_planner test_lab_nav scu_tdt_nav_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    if(OpenCV_FOUND)
        install(TARGETS play RUNTIME DESTINATION lib/${PROJECT_NAME})
    endif()
    install(DIRECTORY config maps tools images worlds urdf launch rviz
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL
    )
    ament_package()
endif()
