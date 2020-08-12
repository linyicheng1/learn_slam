find_package(OpenCV REQUIRED)

include_directories(
        ../common
        ../thirdPart_lib/
        ${OpenCV_INCLUDE_DIRS}
)

add_library(
        fast
        extract.cpp 
        extract_fast.cpp
        feature_alignment.cpp
        feature_matcher.cpp
)

target_link_libraries(fast
        common
        ${OpenCV_LIBS}
        ${CMAKE_SOURCE_DIR}/thirdPart_lib/fast/bin/libfast.so
)

add_library(
        orb
        extract.cpp 
        extract_orb.cpp
)

target_link_libraries(orb
        ${OpenCV_LIBS}
)

