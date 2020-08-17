cmake_minimum_required(VERSION 3.5)
project(my_slam)
set(CMAKE_CXX_STANDARD 11)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)

add_subdirectory(./imu)
add_subdirectory(./cam)
add_subdirectory(./feature_extract)
add_subdirectory(./depth_filter)

include_directories(
        ./common
        ./test
        ./imu
        ./cam 
        ./feature_extract 
        /usr/include/eigen3
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_library(
        common
        ./common/ground_truth.cpp
        ./common/visualization.cpp 
        ./common/graph_base/feature2d.cpp 
        ./common/graph_base/frame.cpp
        ./common/graph_base/point3d.cpp
)
target_link_libraries(common
        ${Pangolin_LIBRARIES}
        -lpthread
        -lboost_system
)

################### test ########################

add_executable(test_imu
        ./test/test_imu.cpp
)

target_link_libraries(test_imu
        imu
        common
)

add_executable(test_cam
        ./test/test_cam.cpp
)

target_link_libraries(test_cam
        cam
        common
        ${OpenCV_LIBS}
        
)        
add_executable(test_feature
        ./test/test_feature.cpp
)

target_link_libraries(test_feature
        fast
        orb 
        common
        cam 
        ${OpenCV_LIBS}
        
)   

add_executable(test_filter
        ./test/test_filter.cpp
)

target_link_libraries(test_filter
        fast
        orb 
        common
        cam
        depth_filter
        ${OpenCV_LIBS}
)      