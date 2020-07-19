find_package(OpenCV REQUIRED)

include_directories(
        ../common
)
add_library(
        cam
        cam.cpp
)

target_link_libraries(cam
        ${OpenCV_LIBS}
)
