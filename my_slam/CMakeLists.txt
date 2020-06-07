cmake_minimum_required(VERSION 3.5)
project(my_slam)

set(CMAKE_CXX_STANDARD 11)
add_subdirectory(./imu)
include_directories(
        ./common
        ./test
        ./imu
)

add_executable(test_imu
        ./test/test_imu.cpp
        )

target_link_libraries(test_imu
        imu
        )