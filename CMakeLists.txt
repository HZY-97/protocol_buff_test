cmake_minimum_required(VERSION 3.10.0)
project(protocol VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL CONFIG REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(zstd-1.5.7/build/cmake)

# 包含生成proto文件的子模块
add_subdirectory(protoFile)

# 包含子目录（可以包含多个）
add_subdirectory(src)