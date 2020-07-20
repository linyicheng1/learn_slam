#ifndef __GROUND_TRUTH_H
#define __GROUND_TRUTH_H

#include <vector>
#include <string>
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>

namespace my_slam
{
    class ground_truth_EuRoC
    {
    public:
        ground_truth_EuRoC(std::string path);
        std::vector<Eigen::Vector3f> get_pos(void) const
        {
            return pos_;
        }
        std::vector<Eigen::Quaternionf> get_quaternion(void) const
        {
            return q_;
        }
        std::vector<float> get_time_stamp() const
        {
            return time_stamp_;
        }
    private:
        std::string path_;
        std::vector<Eigen::Vector3f> pos_;
        std::vector<Eigen::Quaternionf> q_;
        std::vector<float> time_stamp_;
    };
};

#endif //__GROUND_TRUTH_H