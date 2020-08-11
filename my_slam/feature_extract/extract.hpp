#ifndef __FEATURE_EXTRACT_H
#define __FEATURE_EXTRACT_H

#include "../common/graph_base/feature2d.hpp"
#include <vector>
#include "common.hpp"

namespace my_slam
{
    

    class feature_extract_config
    {
    public:
        int grid_n_cols_;
        int grid_n_rows_;
        int levels_;
        int cell_size_;
        float focal_length_;
        feature_extract_config() = default;
        feature_extract_config(int grid_n_cols,
                               int grid_n_rows,
                               int levels,
                               int cell_size,
                               float focal_length):
        grid_n_cols_(grid_n_cols),
        grid_n_rows_(grid_n_rows),
        levels_(levels),
        cell_size_(cell_size),
        focal_length_(focal_length)
        {

        }
    };
    class feature_extract
    {
    public:
        feature_extract() = default;
        feature_extract(int grid_n_cols,
                        int grid_n_rows,
                        int levels,
                        int cell_size):
        grid_n_cols_(grid_n_cols),
        grid_n_rows_(grid_n_rows),
        levels_(levels),
        cell_size_(cell_size)
        {

        }   
        ~feature_extract() = default;
        virtual std::vector<feature2d> extract(pic_byte* img, int img_width, int img_height,int level,frame* Frame) = 0;
        virtual std::vector<feature2d> extract(ImgPyr pyramid,frame* Frame) = 0;

        //最大角点数量 grid_n_cols_*grid_n_rows_
        int grid_n_cols_;
        int grid_n_rows_;
        int levels_;
        int cell_size_;
    };

};




#endif //__FEATURE_EXTRACT_H
