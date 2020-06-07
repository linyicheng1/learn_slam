#include "../imu/imu.hpp"
#include <iostream>

using namespace my_slam;

int main()
{
    imu *IMU = new LpSensor("/dev/ttyUSB0");
    float time = IMU->get_time_stamp();
    while(1)
    {
        if(IMU->get_time_stamp()!=time)
        {
            auto acc = IMU->get_raw_acc();
            auto gyro = IMU->get_raw_gyro();
            std::cout<<"acc"<<acc.at(0)<<" "<<acc.at(1)<<" "<<acc.at(2)<<std::endl;
            std::cout<<"gyro"<<gyro.at(0)<<" "<<gyro.at(1)<<" "<<gyro.at(2)<<std::endl;
            time = IMU->get_time_stamp();
        }
    }
}