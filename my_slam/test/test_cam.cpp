#include "../cam/cam.hpp"
#include <iostream>
#include "ground_truth.h"
#include "visualization.h"

using namespace my_slam;

#ifdef EUROC
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
#endif

int main()
{
    depth_data test("/home/lyc/dataSet/test_data/");

    ground_truth_depth gt("/home/lyc/dataSet/test_data/");
    visualization vis;
    vis.set_pos(gt.get_pos());
    vis.set_quaternion(gt.get_quaternion());
    int cnt = 0;
    while (cnt<200)
    {
        cv::Mat cam = test.getFrame();
        if(!cam.empty())
        {
            std::cout<<"cnt:"<<cnt<<std::endl;
            cv::imshow("cam",cam);
        }
        cv::waitKey(200);
        cnt++;
    }
    return 1;
}
