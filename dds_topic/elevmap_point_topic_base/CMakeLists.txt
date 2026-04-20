# 车体高程点云 DDS 类型：IDL 变更后在本目录执行 fastddsgen -replace elevmap_point_topic_base.idl

cmake_minimum_required(VERSION 3.20)
project("generated_code")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "Configuring elevmap_point_topic_base...")
add_library(elevmap_point_topic_base_lib
    elevmap_point_topic_baseTypeObjectSupport.cxx
    elevmap_point_topic_basePubSubTypes.cxx
)
target_compile_definitions(elevmap_point_topic_base_lib PRIVATE $<$<BOOL:${BUILD_SHARED_LIBS}>:EPROSIMA_USER_DLL_EXPORT>)
target_link_libraries(elevmap_point_topic_base_lib PUBLIC fdd_bridge_fastcdr fdd_bridge_fastdds)
