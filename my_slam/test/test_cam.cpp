#include "../cam/cam.hpp"
#include <iostream>
#include "ground_truth.h"
#include "visualization.h"

using namespace my_slam;

int main()
{
    EuRoC *test;
    std::string path = "/home/lyc/dataBase/MH_01_easy/mav0";
    test = new EuRoC(path);
    ground_truth_EuRoC gt("/home/lyc/dataBase/MH_01_easy/mav0");
    
    visualization vis;
    vis.set_pos(gt.get_pos());
    vis.set_quaternion(gt.get_quaternion());
    while (true)
    {
        cv::Mat cam_0,cam_1;
        cam_0 = test->getFrame();
        cam_1 = test->getFrame2();
        cv::imshow("cam 0 ",cam_0);
        cv::imshow("cam 1 ",cam_1);
        cv::waitKey(30);
    }
}