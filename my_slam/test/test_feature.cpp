#include "extract_fast.hpp"
#include "./graph_base/feature2d.hpp"
#include "../cam/cam.hpp"
#include <iostream>
#include "ground_truth.h"
#include "visualization.h"
#include "extract_orb.hpp"

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

    ///////////////////////////////////////////
    feature_extract_config  config;
    extract_fast *test_fast = new extract_fast(config);
    // extract_orb * test_orb = new extract_orb(config);
    while (true)
    {
        cv::Mat cam_0,cam_1;
        cam_0 = test->getFrame();
        cam_1 = test->getFrame2();
        cv::imshow("cam 0 ",cam_0);
        cv::imshow("cam 1 ",cam_1);
        
        std::vector<feature2d> points = test_fast->extract(cam_0.data,cam_0.cols,cam_0.rows);
        cv::Mat show = cam_0.clone();
        for(auto it:points)
        {
            cv::circle(show,cv::Point(it.x_,it.y_), 1,cv::Scalar(255,0,0),-1); 
        }
        cv::imshow("fast",show);

        // std::vector<feature2d> orb_points = test_orb->extract(cam_0);
        // cv::Mat show2 = cam_0.clone();
        // for(auto it:orb_points)
        // {
        //     cv::circle(show2,cv::Point(it.x_,it.y_), 3,cv::Scalar(255,0,0),-1); 
        // }
        // cv::imshow("orb",show2);

        cv::waitKey(30);
    }
}