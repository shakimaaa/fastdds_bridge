cmake_minimum_required(VERSION 3.20)
project(base_fastdds_pubsub)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 父工程已注册 fdd_bridge_fastcdr / fdd_bridge_fastdds（避免与 ROS fastrtps 的 CMake 目标冲突）

# 添加可执行文件
add_library(base_fastdds_pubsub
        src/PublisherApp.cpp
        src/SubscriberApp.cpp
)

# 链接库到目标可执行文件
# 链上 foonathan 后，GNU ld 默认 --as-needed 会因可执行文件未直接引用其符号而省略 DT_NEEDED，间接依赖仍无法解析。
target_link_libraries(base_fastdds_pubsub PUBLIC
        fdd_bridge_fastcdr
        fdd_bridge_fastdds
        "-Wl,--push-state"
        "-Wl,--no-as-needed"
        fdd_bridge_foonathan_memory
        "-Wl,--pop-state"
        motor_state_12_lib
        motor_command_12_lib
        imu_topic_base_lib
        motor_state_16_lib
        motor_command_16_lib
        image_topic_base_lib
)

target_include_directories(base_fastdds_pubsub PUBLIC
        include
        ../dds_topic/motor_12/motor_state_12
        ../dds_topic/motor_12/motor_command_12
        ../dds_topic/motor_16/motor_state_16
        ../dds_topic/motor_16/motor_command_16
        ../dds_topic/imu_topic_base
        ../dds_topic/image_topic_base
        ../dds_topic/elevmap_topic_base
        ../dds_topic/elevmap_point_topic_base
        ../dds_topic/odom_topic_base
)

target_link_libraries(base_fastdds_pubsub PUBLIC
        elevmap_topic_base_lib
        elevmap_point_topic_base_lib
        odom_topic_base_lib
)