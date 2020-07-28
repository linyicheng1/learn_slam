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
        feature_extract_config() = default;
        feature_extract_config(int grid_n_cols,
                               int grid_n_rows,
                               int levels):
        grid_n_cols_(grid_n_cols),
        grid_n_rows_(grid_n_rows),
        levels_(levels)                      
        {

        }
    };
    class feature_extract
    {
    public:
        feature_extract() = default;
        feature_extract(int grid_n_cols,
                        int grid_n_rows,
                        int levels):
        grid_n_cols_(grid_n_cols),
        grid_n_rows_(grid_n_rows),
        levels_(levels)
        {

        }   
        ~feature_extract() = default;
        virtual std::vector<feature2d> extract(pic_byte* img, int img_width, int img_height) = 0;

        //最大角点数量 grid_n_cols_*grid_n_rows_
        int grid_n_cols_;
        int grid_n_rows_;
        int levels_;
    };

};




#endif //__FEATURE_EXTRACT_H