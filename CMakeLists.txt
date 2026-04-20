cmake_minimum_required(VERSION 3.8)
project(fastdds_ros2_bridge)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 不用 find_package(fastdds)：其导出的 CMake 目标（fastdds、eProsima_atomic）会与 ROS Humble 的 fastrtps
# 已注册目标冲突。改为仅查找库文件并创建独立 IMPORTED 目标。
set(FASTDDS_INSTALL_PREFIX "" CACHE PATH "自编译 Fast-DDS 3 + Fast-CDR 2 的安装前缀（含 include/ 与 lib/）")
if(NOT FASTDDS_INSTALL_PREFIX AND DEFINED ENV{FASTDDS_INSTALL_PREFIX})
  set(FASTDDS_INSTALL_PREFIX "$ENV{FASTDDS_INSTALL_PREFIX}")
endif()
if(NOT FASTDDS_INSTALL_PREFIX)
  set(_fdd_guess "${CMAKE_CURRENT_SOURCE_DIR}/../env/fast_dds/local")
  if(EXISTS "${_fdd_guess}/include/fastdds/dds/domain/DomainParticipant.hpp")
    set(FASTDDS_INSTALL_PREFIX "${_fdd_guess}")
  endif()
endif()
if(NOT FASTDDS_INSTALL_PREFIX)
  message(FATAL_ERROR "请设置 FASTDDS_INSTALL_PREFIX（或环境变量 FASTDDS_INSTALL_PREFIX）指向自编译安装根目录，例如 -DFASTDDS_INSTALL_PREFIX=/workspace/src/env/fast_dds/local")
endif()

find_library(FDD_BRIDGE_FASTCDR_LIB NAMES fastcdr REQUIRED HINTS "${FASTDDS_INSTALL_PREFIX}/lib" "${FASTDDS_INSTALL_PREFIX}/lib64")
find_library(FDD_BRIDGE_FASTDDS_LIB NAMES fastdds REQUIRED HINTS "${FASTDDS_INSTALL_PREFIX}/lib" "${FASTDDS_INSTALL_PREFIX}/lib64")
find_path(FDD_BRIDGE_VENDOR_INCLUDE_DIR NAMES fastdds/dds/domain/DomainParticipant.hpp REQUIRED HINTS "${FASTDDS_INSTALL_PREFIX}/include")

add_library(fdd_bridge_fastcdr SHARED IMPORTED GLOBAL)
set_target_properties(fdd_bridge_fastcdr PROPERTIES
  IMPORTED_LOCATION "${FDD_BRIDGE_FASTCDR_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${FASTDDS_INSTALL_PREFIX}/include")

add_library(fdd_bridge_fastdds SHARED IMPORTED GLOBAL)
set_target_properties(fdd_bridge_fastdds PROPERTIES
  IMPORTED_LOCATION "${FDD_BRIDGE_FASTDDS_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${FASTDDS_INSTALL_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES fdd_bridge_fastcdr)

# fastdds 依赖 foonathan_memory；若仅间接链接，可执行文件的 RUNPATH 不会用于解析该 .so（ld.so 规则）。
# 作为直接 NEEDED 链接后，与 fastcdr/fastdds 相同，可从 FASTDDS_INSTALL_PREFIX/lib 被找到。
find_library(FDD_BRIDGE_FOONATHAN_MEMORY_LIB
  NAMES foonathan_memory-0.7.3 foonathan_memory
  HINTS "${FASTDDS_INSTALL_PREFIX}/lib" "${FASTDDS_INSTALL_PREFIX}/lib64")
if(NOT FDD_BRIDGE_FOONATHAN_MEMORY_LIB)
  message(FATAL_ERROR "在 FASTDDS_INSTALL_PREFIX 下未找到 libfoonathan_memory（Fast-DDS 依赖），当前前缀: ${FASTDDS_INSTALL_PREFIX}")
endif()
add_library(fdd_bridge_foonathan_memory SHARED IMPORTED GLOBAL)
set_target_properties(fdd_bridge_foonathan_memory PROPERTIES
  IMPORTED_LOCATION "${FDD_BRIDGE_FOONATHAN_MEMORY_LIB}")

# 必须用 CONFIG：默认 find_package(yaml-cpp) 可能先走 Find 模块，没有 yaml-cpp::yaml-cpp 目标
find_package(yaml-cpp QUIET CONFIG PATHS "${FASTDDS_INSTALL_PREFIX}/lib/cmake/yaml-cpp")
if(NOT TARGET yaml-cpp::yaml-cpp)
  find_package(yaml-cpp REQUIRED CONFIG)
endif()
if(NOT TARGET yaml-cpp::yaml-cpp)
  find_library(_fdd_bridge_yaml_cpp_lib NAMES yaml-cpp REQUIRED HINTS "${FASTDDS_INSTALL_PREFIX}/lib" "${FASTDDS_INSTALL_PREFIX}/lib64")
  find_path(_fdd_bridge_yaml_cpp_inc yaml-cpp/yaml.h REQUIRED HINTS "${FASTDDS_INSTALL_PREFIX}/include")
  add_library(yaml-cpp::yaml-cpp UNKNOWN IMPORTED GLOBAL)
  set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
    IMPORTED_LOCATION "${_fdd_bridge_yaml_cpp_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${_fdd_bridge_yaml_cpp_inc}")
endif()

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_subdirectory(dds_topic/image_topic_base)
add_subdirectory(dds_topic/imu_topic_base)

add_subdirectory(dds_topic/elevmap_topic_base)
add_subdirectory(dds_topic/elevmap_point_topic_base)
add_subdirectory(dds_topic/odom_topic_base)
add_subdirectory(dds_topic/motor_12/motor_command_12)
add_subdirectory(dds_topic/motor_12/motor_state_12)
add_subdirectory(base_fastdds_pubsub)
add_subdirectory(dds_topic/motor_16/motor_command_16)
add_subdirectory(dds_topic/motor_16/motor_state_16)

# 配置与生成的 config.h 均在功能包内 config/ 目录
set(THIS_COM_CONFIG "${PROJECT_SOURCE_DIR}/config/")

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/domain_config.h.cmake
        ${PROJECT_SOURCE_DIR}/config/include/config.h
        @ONLY
)

# 包含生成的IDL类型支持的头文件（与 image/imu 一致，桥接无单独 include）
set(_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${THIS_COM_CONFIG}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/dds_topic/motor_12
        ${CMAKE_CURRENT_SOURCE_DIR}/dds_topic/motor_16
        ${CMAKE_CURRENT_SOURCE_DIR}/dds_topic/imu_topic_base
        ${CMAKE_CURRENT_SOURCE_DIR}/dds_topic/image_topic_base
        ${CMAKE_CURRENT_SOURCE_DIR}/dds_topic/elevmap_topic_base
        ${CMAKE_CURRENT_SOURCE_DIR}/dds_topic/elevmap_point_topic_base
        ${CMAKE_CURRENT_SOURCE_DIR}/dds_topic/odom_topic_base
)
include_directories(${_INCLUDE_DIRS})

# 添加桥接IMU可执行文件
add_executable(fastdds_ros2_bridge_imu_node
        src/fastdds_ros2_bridge_imu.cpp
)
add_executable(fastdds_ros2_bridge_imu_worker
        src/fastdds_ros2_bridge_imu_worker.cpp
)

# 添加桥接图像可执行文件
add_executable(fastdds_ros2_bridge_image_node
        src/fastdds_ros2_bridge_image.cpp
)
add_executable(fastdds_ros2_bridge_image_worker
        src/fastdds_ros2_bridge_image_worker.cpp
)

# elevation_mapping 桥接节点（GridMap -> DDS）
add_executable(fastdds_ros2_bridge_elevation_node
        src/fastdds_ros2_bridge_elevmap.cpp
)
add_executable(fastdds_ros2_bridge_elevation_worker
        src/fastdds_ros2_bridge_elevmap_worker.cpp
)

# 车体高程点云（PointCloud2 -> DDS）
add_executable(fastdds_ros2_bridge_elevmap_body_cloud_node
        src/fastdds_ros2_bridge_elevmap_body_cloud.cpp
)
add_executable(fastdds_ros2_bridge_elevmap_body_cloud_worker
        src/fastdds_ros2_bridge_elevmap_body_cloud_worker.cpp
)

# Odometry 桥接节点（nav_msgs/Odometry -> DDS）
add_executable(fastdds_ros2_bridge_odom_node
        src/fastdds_ros2_bridge_odom.cpp
)
add_executable(fastdds_ros2_bridge_odom_worker
        src/fastdds_ros2_bridge_odom_worker.cpp
)

# 添加桥接关节可执行文件
add_executable(fastdds_ros2_bridge_joint_node
        src/fastdds_ros2_bridge_joint.cpp
)

# 机器人 DDS 子进程：仅链接 FASTDDS_INSTALL_PREFIX（如 ../env/fast_dds/local），避免与 ROS rmw 的 libfastrtps 同进程符号冲突
add_executable(tf_robot_date_dds_worker
        src/tf_robot_date_dds_worker.cpp
)

# ROS 侧父进程：仅 rclcpp/rmw，通过管道接收子进程转发的关节与 IMU
add_executable(tf_robot_date_real
        src/tf_robot_date.cpp
)

target_link_libraries(fastdds_ros2_bridge_imu_node
        ament_index_cpp::ament_index_cpp
)
target_link_libraries(fastdds_ros2_bridge_imu_worker
        yaml-cpp::yaml-cpp
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        imu_topic_base_lib
        elevmap_topic_base_lib
        odom_topic_base_lib
        base_fastdds_pubsub
)

target_link_libraries(fastdds_ros2_bridge_joint_node
        yaml-cpp::yaml-cpp
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        motor_state_12_lib
        motor_command_12_lib
        motor_state_16_lib
        motor_command_16_lib
        elevmap_topic_base_lib
        odom_topic_base_lib
        base_fastdds_pubsub
)

target_link_libraries(fastdds_ros2_bridge_elevation_node
        ament_index_cpp::ament_index_cpp
)
target_link_libraries(fastdds_ros2_bridge_elevation_worker
        yaml-cpp::yaml-cpp
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        elevmap_topic_base_lib
        base_fastdds_pubsub
)

target_link_libraries(fastdds_ros2_bridge_elevmap_body_cloud_node
        ament_index_cpp::ament_index_cpp
)
target_link_libraries(fastdds_ros2_bridge_elevmap_body_cloud_worker
        yaml-cpp::yaml-cpp
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        elevmap_point_topic_base_lib
        base_fastdds_pubsub
)

target_link_libraries(fastdds_ros2_bridge_odom_node
        ament_index_cpp::ament_index_cpp
)
target_link_libraries(fastdds_ros2_bridge_odom_worker
        yaml-cpp::yaml-cpp
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        odom_topic_base_lib
        base_fastdds_pubsub
)

target_link_libraries(fastdds_ros2_bridge_image_node
        ament_index_cpp::ament_index_cpp
)
target_link_libraries(fastdds_ros2_bridge_image_worker
        yaml-cpp::yaml-cpp
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        image_topic_base_lib
        elevmap_topic_base_lib
        odom_topic_base_lib
        base_fastdds_pubsub
)

target_link_libraries(tf_robot_date_dds_worker
        yaml-cpp::yaml-cpp
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        motor_state_12_lib
        motor_command_12_lib
        motor_state_16_lib
        motor_command_16_lib
        imu_topic_base_lib
        image_topic_base_lib
        elevmap_topic_base_lib
        odom_topic_base_lib
        base_fastdds_pubsub
)

target_link_libraries(tf_robot_date_real
        ament_index_cpp::ament_index_cpp
)

# 为目标增加依赖项
ament_target_dependencies(fastdds_ros2_bridge_image_node
        rclcpp
        sensor_msgs
        std_msgs
)

# 为目标增加依赖项
ament_target_dependencies(fastdds_ros2_bridge_imu_node
        rclcpp
        sensor_msgs
        std_msgs
)

ament_target_dependencies(fastdds_ros2_bridge_elevation_node
        rclcpp
        sensor_msgs
        std_msgs
        grid_map_msgs
)

ament_target_dependencies(fastdds_ros2_bridge_elevmap_body_cloud_node
        rclcpp
        sensor_msgs
        std_msgs
)

ament_target_dependencies(fastdds_ros2_bridge_odom_node
        rclcpp
        sensor_msgs
        std_msgs
        nav_msgs
)

# 为目标增加依赖项
ament_target_dependencies(fastdds_ros2_bridge_joint_node
        rclcpp
        sensor_msgs
        std_msgs
)
# 为目标增加依赖项
ament_target_dependencies(tf_robot_date_real
        rclcpp
        tf2
        tf2_ros
        geometry_msgs
        sensor_msgs
)

# 安装可执行文件
install(TARGETS
        fastdds_ros2_bridge_image_node
        fastdds_ros2_bridge_image_worker
        fastdds_ros2_bridge_imu_node
        fastdds_ros2_bridge_imu_worker
        fastdds_ros2_bridge_elevation_node
        fastdds_ros2_bridge_elevation_worker
        fastdds_ros2_bridge_elevmap_body_cloud_node
        fastdds_ros2_bridge_elevmap_body_cloud_worker
        fastdds_ros2_bridge_odom_node
        fastdds_ros2_bridge_odom_worker
        fastdds_ros2_bridge_joint_node
        tf_robot_date_dds_worker
        tf_robot_date_real
        DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(
        DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)
# 包声明
ament_package()