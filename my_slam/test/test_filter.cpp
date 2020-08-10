#include "extract_fast.hpp"
#include "./graph_base/feature2d.hpp"
#include "../cam/cam.hpp"
#include <iostream>
#include "ground_truth.h"
#include "visualization.h"
#include "extract_orb.hpp"
#include <time.h>
#include "../common/graph_base/frame.hpp"

using namespace my_slam;

int main()
{
    depth_data test("/home/lyc/dataSet/test_data/");
    ground_truth_depth gt("/home/lyc/dataSet/test_data/");
    
    visualization vis;
    vis.set_pos(gt.get_pos());
    vis.set_quaternion(gt.get_quaternion());

    ///////////////////////////////////////////
    feature_extract_config  config;
    config.grid_n_cols_ = 40;
    config.grid_n_rows_ = 30;
    config.levels_ = 1;
    config.cell_size_ = 16;
    extract_fast *test_fast = new extract_fast(config);

    int cnt = 0;
    double time = 0;
    while (cnt<200)
    {
        cnt ++;
        cv::Mat cam_0;
        cam_0 = test.getFrame();
        if(cam_0.empty())
        {
            continue;
        }
        cv::cvtColor(cam_0,cam_0,CV_BGR2GRAY);
        cv::imshow("cam 0 ",cam_0);

        //std::cout<<"cols"<<cam_0.cols<<"rows"<<cam_0.rows;
        clock_t start,end;
        start = clock();
        frame *test_frame = new frame(cam_0.data,cam_0.cols,cam_0.rows,2);
        std::vector<feature2d> points = test_fast->extract(test_frame->pyramid_,test_frame);
        end = clock();
        time += double(end-start)/CLOCKS_PER_SEC;
        ///////////////////////////////////////
        cv::Mat show = cam_0.clone();
        cv::Mat show1(test_frame->pyramid_[0].rows,test_frame->pyramid_[0].cols,cam_0.type());
        cv::Mat show2(test_frame->pyramid_[1].rows,test_frame->pyramid_[1].cols,cam_0.type());
       // cv::Mat show3(test_frame->pyramid_[2].rows,test_frame->pyramid_[2].cols,cam_0.type());
//        cv::Mat show4(test_frame->pyramid_[3].rows,test_frame->pyramid_[3].cols,cam_0.type());
        show1.data = test_frame->pyramid_[0].data;
        show2.data = test_frame->pyramid_[1].data;
      //  show3.data = test_frame->pyramid_[2].data;
//        show4.data = test_frame->pyramid_[3].data;

        int stride = cam_0.step.p[0];
        for(auto it:points)
        {
            switch (it.level_)
            {
            case 0:
                cv::circle(show1,cv::Point(it.x_,it.y_), 2,cv::Scalar(255,0,0),-1);
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(255,0,0),-1);
                break;
            case 1:
                cv::circle(show2,cv::Point(it.x_/2,it.y_/2), 2,cv::Scalar(0,255,0),-1);
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(0,255,0),-1);
                break;
            case 2:
                //cv::circle(show3,cv::Point(it.x_/4,it.y_/4), 2,cv::Scalar(0,0,255),-1);
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(0,0,255),-1);
                break;
            case 3:
                //cv::circle(show4,cv::Point(it.x_/8,it.y_/8), 2,cv::Scalar(255,255,255),-1);
                cv::circle(show,cv::Point(it.x_,it.y_), 2,cv::Scalar(255,255,255),-1);
                break;
            default:
                break;
            }
        }
//        cv::imshow("fast",show);
//        cv::imshow("pyramid 1",show1);
//        cv::imshow("pyramid 2",show2);
//        cv::imshow("pyramid 3",show3);
//        cv::imshow("pyramid 4",show4);
//        cv::waitKey(100);
    }
    std::cout<<"cost time:"<<time*5<<"ms"<<std::endl;
}