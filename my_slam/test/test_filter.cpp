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

class test_filter: public sparse_depth_filter
{
public:
    test_filter(feature_extract_config config):
            sparse_depth_filter(config)
    {
    }
    cv::Mat view_search_line()
    {
        frame* frame = get_frame();
        cv::Mat pic(frame->pyramid_[0].rows,frame->pyramid_[0].cols,0);
        pic.data = frame->pyramid_[0].data;
        cv::Mat show = pic.clone();

        auto pts_B = get_matcher().pts_B;
        auto pts_A = get_matcher().pts_A;
        assert(pts_A.size()==pts_B.size());
        //for(int i=0;i<pts_A.size();i++)
        if(pts_A.size()>200)
        {
            cv::Point pt1((int)pts_A.at(200).x(),(int)pts_A.at(200).y());
            cv::Point pt2((int)pts_B.at(200).x(),(int)pts_B.at(200).y());
            cv::line(show, pt1, pt2, cv::Scalar(0, 255, 255), 1);
        }
        return show;
    }
    cv::Mat view_match_result()
    {
//        assert(!pts.empty());
//        for(const auto& pt:pts)
//        {
//            cv::circle(pic, pt, 1, cv::Scalar(0, 255, 0), -1);
//        }
//        return pic;
    }
    cv::Mat view_kf_with_feature()
    {
        frame* kf = get_kf();
        cv::Mat pic(kf->pyramid_[0].rows,kf->pyramid_[0].cols,0);
        pic.data = kf->pyramid_[0].data;
        cv::Mat show = pic.clone();
        std::vector<feature2d> features = get_extract()->get_features();
        //for(const auto& feature:features)
        {
            cv::circle(show, cv::Point(features[200].x_,features[200].y_), 5, cv::Scalar(0, 0, 255), -1);
        }
        return show;
    }
};
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
    auto *filter = new test_filter(config);
    filter->set_cam((camera*)&test);
    int cnt = 0;
    double time = 0;
    cv::Mat kf,search_line,search_result;
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
        filter->add_frame(test_frame);
        vis.set_depth_map(filter->get_depth_map());

        end = clock();
        time += double(end-start)/CLOCKS_PER_SEC;

        /////////////////////////////////////
        cv::imshow("key_frame",filter->view_kf_with_feature());
        cv::imshow("search line",filter->view_search_line());
        //cv::imshow("search line with result",filter->view_match_result());
        cv::imshow("pic",cam_0);
        cv::waitKey(300);
    }
    std::cout<<"cost time:"<<time*5<<"ms"<<std::endl;
}