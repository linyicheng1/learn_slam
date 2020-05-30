#ifndef __MY_ORB_H
#define __MY_ORB_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

namespace my_cv
{
    void ORB_detect(cv::Mat image,std::vector<cv::KeyPoint>& keypoints);
}
#endif //__MY_ORB_H

