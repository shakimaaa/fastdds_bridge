# Odometry 话题 DDS 类型：需先在本目录执行 fastddsgen -replace odom_topic_base.idl 生成 .hpp/.cxx

cmake_minimum_required(VERSION 3.20)
project("generated_code")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "Configuring odom_topic_base...")
add_library(odom_topic_base_lib odom_topic_baseTypeObjectSupport.cxx odom_topic_basePubSubTypes.cxx)
target_compile_definitions(odom_topic_base_lib PRIVATE $<$<BOOL:${BUILD_SHARED_LIBS}>:EPROSIMA_USER_DLL_EXPORT>)
target_link_libraries(odom_topic_base_lib fdd_bridge_fastcdr fdd_bridge_fastdds)
