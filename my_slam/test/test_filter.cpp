#include "extract_fast.hpp"
#include "./graph_base/feature2d.hpp"
#include "../cam/cam.hpp"
#include <iostream>
#include "ground_truth.h"
#include "visualization.h"
#include "extract_orb.hpp"
#include "../common/graph_base/frame.hpp"

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
    config.grid_n_cols_ = 24;
    config.grid_n_rows_ = 15;
    config.levels_ = 1;
    config.cell_size_ = 32;
    extract_fast *test_fast = new extract_fast(config);
    // extract_orb * test_orb = new extract_orb(config);
    int cnt = 0;
    while (true)
    {
        cnt ++;
        if(cnt>=40)
        {
            break;
        }
        cv::Mat cam_0,cam_1;
        cam_0 = test->getFrame();
        cam_1 = test->getFrame2();
        cv::imshow("cam 0 ",cam_0);
        cv::imshow("cam 1 ",cam_1);
        //std::cout<<"cols"<<cam_0.cols<<"rows"<<cam_0.rows;
        frame *test_frame = new frame(cam_0.data,cam_0.cols,cam_0.rows,4);

        std::vector<feature2d> points = test_fast->extract(test_frame->pyramid_,test_frame);
        
        cv::Mat show = cam_0.clone();
        int stride = cam_0.step.p[0];
        for(auto it:points)
        {
            switch (it.level_)
            {
            case 0:
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(255,0,0),-1); 
                break;
            case 1:
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(0,255,0),-1); 
                break;
            case 2:
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(0,0,255),-1); 
                break;
            case 3:
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(255,255,255),-1); 
                break;        
            default:
                break;
            }
        }
        cv::imshow("fast",show);

        // std::vector<feature2d> orb_points = test_orb->extract(cam_0);
        // cv::Mat show2 = cam_0.clone();
        // for(auto it:orb_points)
        // {
        //     cv::circle(show2,cv::Point(it.x_,it.y_), 3,cv::Scalar(255,0,0),-1); 
        // }
        // cv::imshow("orb",show2);

        cv::waitKey(100);
    }
}