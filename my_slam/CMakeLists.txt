cmake_minimum_required(VERSION 3.5)
project(my_slam)
set(CMAKE_CXX_STANDARD 11)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)

add_subdirectory(./imu)
add_subdirectory(./cam)

include_directories(
        ./common
        ./test
        ./imu
        ./cam 
        /usr/include/eigen3
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_library(
        common
        ./common/ground_truth.cpp
        ./common/visualization.cpp 
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