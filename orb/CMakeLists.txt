cmake_minimum_required(VERSION 3.5)
project(orb)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(orb
        orb.cpp
        orb.hpp
)

target_link_libraries(orb
        ${OpenCV_LIBS}
)
