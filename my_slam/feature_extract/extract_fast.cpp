#include "extract_fast.hpp"
#include <algorithm>
#include <stdint.h>
#include <math.h>
#include <vector>
#include <stdio.h>
namespace my_slam
{

    std::vector<feature2d> extract_fast::extract(pic_byte* img, int img_width, int img_height,int level,frame* Frame)
    {   
        std::vector<fast::fast_xy> fast_corners;
        std::vector<feature2d> features;
        const int scale = (1<<level);// scale = 2^{level}


        fast::fast_corner_detect_10_sse2(img,img_width,img_height,img_width,8,fast_corners);
        std::vector<int> scores, nm_corners;
        fast::fast_corner_score_10(img, img_width, fast_corners, 20, scores);
        fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

        for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
        {

            fast::fast_xy& xy = fast_corners.at(*it);
            const int k = static_cast<int>((xy.y*scale)/cell_size_)*grid_n_cols_
                  + static_cast<int>((xy.x*scale)/cell_size_);
            if(grid_occupancy_[k])
                continue;
            const float score = shiTomasiScore(picture(img,img_width,img_height), xy.x, xy.y);
            if(score > corners_->at(k).score)
                corners_->at(k) = Corner(xy.x*scale, xy.y*scale, score, level, 0.0f);
        }
        return std::vector<feature2d>();
    }

    std::vector<feature2d> extract_fast::extract(ImgPyr pyramid,frame* Frame)
    {
        features_.clear();
        // 构造角点结构体vector,设置最大角点数量 grid_n_cols_*grid_n_rows_
        corners_ = new Corners(grid_n_cols_*grid_n_rows_, Corner(0,0,threshold_,0,0.0f));
        for(int i=0;i<pyramid.size();i++)
        {
            extract(pyramid.at(i).data,pyramid.at(i).cols,pyramid.at(i).rows,i,Frame);
        }
        // 对于有足够高的得分的角点构造成特征点
        std::for_each(corners_->begin(), corners_->end(), [&](Corner& c)
        {// 遍历所有的角点
            // 要求得分大于阈值
            if(c.score > threshold_)
            {
                // 构造特征点
                // 1、所属帧 2、特征点位置 3、金字塔层数
                feature2d feature(Frame,c.x,c.y,c.level,c.score,c.angle);
                features_.push_back(feature);
            }
        });
        return features_;
        resetGrid();
    }

    extract_fast::extract_fast(feature_extract_config config):
    config_(config),
    feature_extract(config.grid_n_cols_,config.grid_n_rows_,config.levels_,config.cell_size_),
    grid_occupancy_(config.grid_n_cols_*config.grid_n_rows_, false),
    threshold_(50)
    {
        features_.reserve(config_.grid_n_cols_*config_.grid_n_rows_);
    }

    void extract_fast::resetGrid()
    {
        std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
    }

    float extract_fast::shiTomasiScore(picture img,int u,int v)
    {
        float dXX = 0.0;
        float dYY = 0.0;
        float dXY = 0.0;
        const int halfbox_size = 4;
        const int box_size = 2*halfbox_size;
        const int box_area = box_size*box_size;
        const int x_min = u-halfbox_size;
        const int x_max = u+halfbox_size;
        const int y_min = v-halfbox_size;
        const int y_max = v+halfbox_size;

        if(x_min < 1 || x_max >= img.cols-1 || y_min < 1 || y_max >= img.rows-1)
            return 0.0; // patch is too close to the boundary

        const int stride = img.cols;
        for( int y=y_min; y<y_max; ++y )
        {
            const uint8_t* ptr_left   = img.data + stride*y + x_min - 1;
            const uint8_t* ptr_right  = img.data + stride*y + x_min + 1;
            const uint8_t* ptr_top    = img.data + stride*(y-1) + x_min;
            const uint8_t* ptr_bottom = img.data + stride*(y+1) + x_min;
            for(int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
            {
                float dx = *ptr_right - *ptr_left;
                float dy = *ptr_bottom - *ptr_top;
                dXX += dx*dx;
                dYY += dy*dy;
                dXY += dx*dy;
            }
        }

        // Find and return smaller eigenvalue:
        dXX = dXX / (2.0 * box_area);
        dYY = dYY / (2.0 * box_area);
        dXY = dXY / (2.0 * box_area);
        return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
    }
};