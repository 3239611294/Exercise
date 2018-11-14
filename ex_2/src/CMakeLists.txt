#add_library(hello SHARED hello.cpp)
include_directories( "/usr/include/eigen3" )
add_executable(useeigen useeigen.cpp)
add_executable(rotation rotation.cpp)
#target_link_libraries(useeigen hello)
# 安装库文件
#INSTALL(TARGETS hello LIBRARY DESTINATION lib)

# 安装头文件
#INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION include)

