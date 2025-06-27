file(GLOB_RECURSE ALL_PROTO_FILES
    CONFIGURE_DEPENDS  # 自动检测新文件
    "${CMAKE_CURRENT_SOURCE_DIR}/*.proto"
)

set(ALL_PROTO_SRCS "")
set(ALL_PROTO_HDRS "")

foreach(PROTO_FILE IN LISTS ALL_PROTO_FILES)

    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILE})
    list(APPEND ALL_PROTO_SRCS ${PROTO_SRCS})
    list(APPEND ALL_PROTO_HDRS ${PROTO_HDRS})

endforeach()

# 创建共享库
add_library(common_proto SHARED ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})

# 设置共享的头文件目录
target_include_directories(common_proto PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}  # 生成的 .pb.h 文件位置
    ${Protobuf_INCLUDE_DIRS}     # Protobuf 头文件
)

# 链接 Protobuf 库
target_link_libraries(common_proto PRIVATE ${Protobuf_LIBRARIES})

install(
    TARGETS common_proto
    DESTINATION .
)

set_target_properties(
    common_proto
    PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)