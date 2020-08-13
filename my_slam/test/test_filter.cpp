#include "extract_fast.hpp"
#include "./graph_base/feature2d.hpp"
#include "../cam/cam.hpp"
#include <iostream>
#include "ground_truth.h"
#include "visualization.h"
#include "extract_orb.hpp"
#include <time.h>
#include "../common/graph_base/frame.hpp"
#include "../depth_filter/sparse_depth_filter.hpp"

using namespace my_slam;

int main()
{
    depth_data test("/home/lyc/dataSet/test_data/");
    ground_truth_depth gt("/home/lyc/dataSet/test_data/");

    visualization vis;
    //vis.set_pos(gt.get_pos());
    //vis.set_quaternion(gt.get_quaternion());

    ///////////////////////////////////////////
    feature_extract_config  config{};
    config.grid_n_cols_ = 40;
    config.grid_n_rows_ = 30;
    config.levels_ = 1;
    config.cell_size_ = 16;
    auto *test_filter = new sparse_depth_filter(config);
    test_filter->set_cam((camera*)&test);
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
        clock_t start,end;
        start = clock();
        int id = test.getFrameId();
        auto *test_frame = new frame(cam_0.data,cam_0.cols,cam_0.rows,2);
        test_frame->q_ = gt.get_quaternion()[id];
        test_frame->t_ = gt.get_pos()[id];
        test_filter->add_frame(*test_frame);
        vis.set_depth_map(test_filter->get_depth_filter());

        end = clock();
        time += double(end-start)/CLOCKS_PER_SEC;
        /////////////////////////////////////
        cv::imshow("pic",cam_0);
        cv::waitKey(300);
    }
    std::cout<<"cost time:"<<time*5<<"ms"<<std::endl;
}