cmake_minimum_required(VERSION 3.10.0)
project(protocol VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL CONFIG REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
include_directories(${CMAKE_BINARY_DIR})

# 包含生成proto文件的子模块
add_subdirectory(protoFile)

# 包含子目录（可以包含多个）
add_subdirectory(src)

# set(PROTO_FILE "protoFile/sensor.proto")
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILE})

# add_executable(protocol ${PROTO_SRCS} ${PROTO_HDRS} src/main.cpp)
# target_link_libraries(protocol ${PROTOBUF_LIBRARIES})