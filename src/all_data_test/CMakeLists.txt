
add_executable(all_Serialize2File Serialize2File.cpp)
# 链接proto生成器库（自动包含头文件）
target_link_libraries(all_Serialize2File 
    PRIVATE ${Protobuf_LIBRARIES} 
    PRIVATE common_proto
    PRIVATE libzstd
)


add_executable(all_ParseFromFile ParseFromFile.cpp)
# 链接proto生成器库（自动包含头文件）
target_link_libraries(all_ParseFromFile 
    PRIVATE ${Protobuf_LIBRARIES} 
    PRIVATE common_proto
    PRIVATE ${PCL_LIBRARIES}
    PRIVATE libzstd
)

install(
    TARGETS all_Serialize2File
    DESTINATION .
)

set_target_properties(
    all_Serialize2File
    PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)

install(
    TARGETS all_ParseFromFile
    DESTINATION .
)

set_target_properties(
    all_ParseFromFile
    PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)