#include "feature2d.hpp"

namespace my_slam
{
    long int feature2d::id_ = 0;
    feature2d::feature2d(int x,int y):
    x_(x),
    y_(y)
    {
        init();
    }
    void feature2d::init()
    {
        id_ ++;
    }

};