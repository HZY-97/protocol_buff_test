
add_executable(all_Serialize2File Serialize2File.cpp)
# 链接proto生成器库（自动包含头文件）
target_link_libraries(all_Serialize2File 
    PRIVATE ${Protobuf_LIBRARIES} 
    PRIVATE common_proto
)


add_executable(all_ParseFromFile ParseFromFile.cpp)
# 链接proto生成器库（自动包含头文件）
target_link_libraries(all_ParseFromFile 
    PRIVATE ${Protobuf_LIBRARIES} 
    PRIVATE common_proto
    PRIVATE ${PCL_LIBRARIES}
)
