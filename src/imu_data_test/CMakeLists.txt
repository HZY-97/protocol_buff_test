
add_executable(imu_Serialize2File Serialize2File.cpp)
# 链接proto生成器库（自动包含头文件）
target_link_libraries(imu_Serialize2File 
    PRIVATE ${Protobuf_LIBRARIES} 
    PRIVATE common_proto
    PRIVATE libzstd
)


add_executable(imu_ParseFromFile ParseFromFile.cpp)
# 链接proto生成器库（自动包含头文件）
target_link_libraries(imu_ParseFromFile 
    PRIVATE ${Protobuf_LIBRARIES} 
    PRIVATE common_proto
    PRIVATE ${PCL_LIBRARIES}
    PRIVATE libzstd
)

install(
    TARGETS imu_Serialize2File
    DESTINATION .
)

set_target_properties(
    imu_Serialize2File
    PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)

install(
    TARGETS imu_ParseFromFile
    DESTINATION .
)

set_target_properties(
    imu_ParseFromFile
    PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)