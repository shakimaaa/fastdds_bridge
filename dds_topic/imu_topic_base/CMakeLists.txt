
# Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)

project("generated_code")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set CMAKE_BUILD_TYPE to Release by default.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


message(STATUS "Configuring imu_topic_base...")
add_library(imu_topic_base_lib imu_topic_baseTypeObjectSupport.cxx imu_topic_basePubSubTypes.cxx)
target_link_libraries(imu_topic_base_lib fdd_bridge_fastcdr fdd_bridge_fastdds)

add_executable(imu_topic_base imu_topic_baseApplication.cxx imu_topic_basePublisherApp.cxx imu_topic_baseSubscriberApp.cxx imu_topic_basemain.cxx)
target_link_libraries(imu_topic_base fdd_bridge_fastcdr fdd_bridge_fastdds
        imu_topic_base_lib 
        )



