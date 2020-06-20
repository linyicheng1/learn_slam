cmake_minimum_required( VERSION 2.8 )
project( g2o_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O
find_package( G2O REQUIRED )
include_directories( 
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

add_executable( curve_fitting curve_fit2.cpp )
# 与G2O链接
target_link_libraries( curve_fitting 
    g2o_core g2o_stuff
)
