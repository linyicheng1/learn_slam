#include "extract_fast.hpp"


namespace my_slam
{

    std::vector<feature2d> extract_fast::extract(pic_byte* img, int img_width, int img_height)
    {   
        std::cout<<"extract!"<<std::endl;
        std::vector<fast::fast_xy> fast_corners;
        fast::fast_corner_detect_10_sse2(img,img_width,img_height,img_width,50,fast_corners);
        features_.empty();
        features_.resize(fast_corners.size());
        for(auto it:fast_corners)
        {
            feature2d feature;
            feature.set_feature((int)it.x,(int)it.y);
            features_.push_back(feature);
        }
        return features_;
    }

    extract_fast::extract_fast(feature_extract_config config):
    config_(config),
    feature_extract(config_.grid_n_cols_,config_.grid_n_rows_,config_.levels_)
    {

    }           
};