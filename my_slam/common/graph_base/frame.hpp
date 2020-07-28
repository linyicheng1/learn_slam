#ifndef __FRAME_H
#define __FRAME_H
#include "../common.hpp"

namespace my_slam
{
    class frame
    {
        frame() =default;
        ~frame() = default;
        frame(cv::Mat )
    public:
        ImgPyr pyramid_;
        int key_frame_;
    };
};

#endif // __FRAME_H
