
include_directories(
        ../common
)
add_library(
        imu
        imu.cpp
)

target_link_libraries(
        imu
        LpSensor.so
        pthread
        dl
        rt
)
