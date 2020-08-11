include_directories(
        ../common
)
add_library(
        depth_filter
        sparse_depth_filter.cpp
        matcher.cpp
)