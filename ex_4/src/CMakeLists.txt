#include_directories("usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBRARIES})

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBRARIES})

add_executable(disparity.cpp disparity.cpp)
target_link_libraries(disparity.cpp 
${Pangolin_LIBRARIES}
${OpenCV_LIBRARIES})
# 安装库文件
#INSTALL(TARGETS hello LIBRARY DESTINATION lib)

# 安装头文件
#INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION include)

