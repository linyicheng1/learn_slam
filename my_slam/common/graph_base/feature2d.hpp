#ifndef __FEATURE_2D_H
#define __FEATURE_2D_H

#include "frame.hpp"

namespace my_slam
{
    class frame;

    class feature2d
    {
        static long int id_;
    public:
        feature2d();
        feature2d(frame* Frame,int x,int y,int level,float score,float angle);
        ~feature2d() =default;
        void set_feature(int x,int y){x_ = x;y_=y;}
        void init();
        int x_;
        int y_;
        int level_;
        float score_;
        float angle_;
        long int frame_id_;
        frame* frame_;
    };
};

#endif // __FEATURE_2D_H