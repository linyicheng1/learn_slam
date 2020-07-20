#ifndef __IMU_H
#define __IMU_H
#include <cstdio>
#include <thread>
// #include "lpsensor/LpmsSensorI.h"
// #include "lpsensor/LpmsSensorManagerI.h"
#include "thread.h"
#include <vector>
#include <string>

namespace my_slam
{
// imu的抽象类
    class imu
    {
    public:
        imu();
        ~imu() = default;
        void init();
        virtual void get_sensor_data(const std::vector<float> &acc,
                                     const std::vector<float> &gyro,
                                     const float time) = 0;
        // 部分数据操作接口
        std::vector<float> get_raw_acc() { return raw_acc_data_; }
        std::vector<float> get_raw_gyro() { return raw_gyro_data_; }
 
        float get_time_stamp(){ return time_stamp_;}

    public:
        std::vector<float> raw_acc_data_;
        std::vector<float> raw_gyro_data_;
        float time_stamp_;
    };

// 使用LpSensor作为IMU传感器
    // class LpSensor : public imu,public thread
    // {
    // public:
    //     LpSensor() = default;
    //     ~LpSensor()= default;
    //     LpSensor(std::string dev);

    // private:
    //     void correct_data() override;
    //     void get_sensor_data() override;
    //     LpmsSensorManagerI* manager_;
    //     LpmsSensorI* lpms_;
    //     ImuData data_;
    //     void process() override;
    // };

// 使用数据集作为IMU传感器
    class EuRoC_IMU : public imu
    {
        EuRoC_IMU() = default;
        ~EuRoC_IMU() = default;
        EuRoC_IMU(std::string path);
    private:
        void get_sensor_data(const std::vector<float> &acc,
                            const std::vector<float> &gyro,
                            const float time) override;
        std::string path_;                    
    };
}
#endif // __IMU_H
