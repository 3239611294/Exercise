add_library(hello SHARED hello.cpp)

add_executable(usehello usehello.cpp)
target_link_libraries(usehello hello)
# 安装库文件
INSTALL(TARGETS hello LIBRARY DESTINATION lib)

# 安装头文件
INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION include)

