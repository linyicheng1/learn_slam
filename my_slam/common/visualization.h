#ifndef __VISUALIZATION_H
#define __VISUALIZATION_H

#include <pangolin/pangolin.h>
#include "thread.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace my_slam
{
    class visualization:public thread
    {
    public:
        visualization();
        void set_pos(std::vector<Eigen::Vector3f> pos){pos_ = pos;}
        void set_quaternion(std::vector<Eigen::Quaternionf> q){q_ = q;}
        ~visualization();
    private:
        std::vector<Eigen::Vector3f> pos_;
        std::vector<Eigen::Quaternionf> q_;
        void process() override;
        void draw_trajectory();
        void draw_pose();
        static void* pthread_fun(void* __this);
    };
};




#endif // __VISUALIZATION_H