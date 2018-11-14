include_directories( "/usr/include/eigen3" )
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories( 
${Sophus_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS} )

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory 
${Pangolin_LIBRARIES}
${Sophus_LIBRARIES})

add_executable(contast contast.cpp)
target_link_libraries(contast 
${Pangolin_LIBRARIES}
${Sophus_LIBRARIES})
#target_link_libraries(useeigen hello)
# 安装库文件
#INSTALL(TARGETS hello LIBRARY DESTINATION lib)

# 安装头文件
#INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION include)

