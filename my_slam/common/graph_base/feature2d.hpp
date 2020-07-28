#ifndef __FEATURE_2D_H
#define __FEATURE_2D_H

#include <eigen3/Eigen/Core>
namespace my_slam
{
    class feature2d
    {
        static long int id_;
    public:
        feature2d() = default;
        feature2d(int x,int y);
        ~feature2d() =default;
        void set_feature(int x,int y){x_ = x;y_=y;}
        void init();
        int x_;
        int y_;
    };
};

#endif // __FEATURE_2D_H