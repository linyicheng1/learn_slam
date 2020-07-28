#include "feature2d.hpp"


namespace my_slam
{
    long int feature2d::id_ = 0;
    feature2d::feature2d(frame* Frame,int x,int y,int level,float score,float angle):
    x_(x),
    y_(y),
    level_(level),
    score_(score),
    angle_(angle),
    frame_(Frame)
    {
        init();
    }
    feature2d::feature2d()
    {
        x_ = 0;
        y_ = 0;
        level_ = 0;
        score_ = 0;
        angle_ = 0;
        init();
    }
    void feature2d::init()
    {
        id_ ++;
    }

};