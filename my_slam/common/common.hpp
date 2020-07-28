#ifndef __COMMON_H
#define __COMMON_H
#include <opencv2/opencv.hpp>
#include <vector>

namespace my_slam
{
    typedef std::vector<cv::Mat> ImgPyr;
    typedef unsigned char pic_byte;
};


#endif // __COMMON_H
