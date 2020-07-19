#include "imu.hpp"


namespace my_slam
{
    
    // /**
    //  * @brief 抽象函数imu初始化
    //  */
    // imu::imu()
    // {
    //     init();
    // }
    // void imu::init()
    // {
    //     raw_acc_data_.resize(3);
    //     raw_gyro_data_.resize(3);
    //     bias_acc_.resize(3);
    //     bias_gyro_.resize(3);
    //     correct_acc_.resize(3);
    //     correct_gyro_.resize(3);
    // }

    // /**
    //  * @brief 陀螺仪模块 LpSensor构造函数
    //  * @param dev 设备名称
    //  */
    // LpSensor::LpSensor(std::string dev)
    // {
    //     init();
    //     manager_ = LpmsSensorManagerFactory();
    //     lpms_ = manager_->addSensor(DEVICE_LPMS_RS232, dev.c_str());
    //     start(10);
    // }

    // void LpSensor::correct_data()
    // {

    // }

    // /**
    //  * @brief 获取当前imu数据
    //  */
    // void LpSensor::get_sensor_data()
    // {
    //     // Checks, if sensor is connected
    //     if (lpms_->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
    //         lpms_->hasImuData())
    //     {
    //         // Reads quaternion data
    //         data_ = lpms_->getCurrentData();
    //         for (int i = 0; i < 3; i++)
    //         {
    //             raw_acc_data_.at(i) = data_.a[i];
    //             raw_gyro_data_.at(i) = data_.g[i];
    //             time_stamp_ = (float)data_.timeStamp;
    //         }
    //     }
    // }

    // void LpSensor::process()
    // {
    //     get_sensor_data();
    //     correct_data();
    // }
}