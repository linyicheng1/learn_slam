#include "../cam/cam.hpp"
#include <iostream>

using namespace my_slam;

int main()
{
    EuRoC *test;
    std::string path = "/home/lyc/dataBase/MH_01_easy/mav0";
    test = new EuRoC(path);

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