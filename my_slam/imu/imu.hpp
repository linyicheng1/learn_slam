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
        virtual void correct_data() = 0;
        virtual void get_sensor_data() = 0;
        // 部分数据操作接口
        std::vector<float> get_raw_acc() { return raw_acc_data_; }
        std::vector<float> get_raw_gyro() { return raw_gyro_data_; }
        std::vector<float> get_acc() { return correct_acc_; }
        std::vector<float> get_gyro() { return correct_gyro_; }
        float get_time_stamp(){ return time_stamp_;}
        void set_bias_acc(std::vector<float> bias) { bias_acc_ = bias; }
        void set_bias_gyro(std::vector<float> bias) { bias_gyro_ = bias; }
    public:
        std::vector<float> raw_acc_data_;
        std::vector<float> raw_gyro_data_;
        std::vector<float> bias_acc_;
        std::vector<float> bias_gyro_;
        std::vector<float> correct_acc_;
        std::vector<float> correct_gyro_;
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
    class IMU_data : public imu
    {

    private:
        void correct_data() override;
        void get_sensor_data() override;
    };

}
#endif // __IMU_H
