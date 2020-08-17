#ifndef __VISUALIZATION_H
#define __VISUALIZATION_H

#include <pangolin/pangolin.h>

#include <utility>
#include "thread.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <list>
#include "graph_base/point3d.hpp"
#include "opencv2/opencv.hpp"

namespace my_slam
{
    class visualization:public thread
    {
    public:
        visualization();
        void set_pos(std::vector<Eigen::Vector3f> pos){pos_ = std::move(pos);}
        void set_quaternion(std::vector<Eigen::Quaternionf> q){q_ = std::move(q);}
        void set_depth_map(std::list<point3d> depth_map){depth_map_=std::move(depth_map);}

        ~visualization();
    private:
        std::vector<Eigen::Vector3f> pos_;
        std::vector<Eigen::Quaternionf> q_;
        std::list<point3d> depth_map_;
        void process() override;
        void draw_trajectory();
        void draw_pose();
        void draw_depth_map();
        static void* pthread_fun(void* __this);
    };
};




#endif // __VISUALIZATION_H