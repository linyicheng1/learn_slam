cmake_minimum_required(VERSION 3.5)
project(my_slam)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 11)
add_subdirectory(./imu)
add_subdirectory(./cam)

include_directories(
        ./common
        ./test
        ./imu
        ./cam 
        
        ${OpenCV_INCLUDE_DIRS}
)

add_library(
        common
        ./common/ground_truth.cpp
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