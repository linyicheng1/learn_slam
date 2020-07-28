#ifndef __EXTRACY_FAST_H
#define __EXTRACY_FAST_H

#include "extract.hpp"
#include <vector>
#include "../thirdPart_lib/fast/include/fast/fast.h"
#include "common.hpp"

namespace my_slam
{
    class extract_fast:public feature_extract
    {   
    public:
        extract_fast() = default;
        ~extract_fast() = default;
        extract_fast(feature_extract_config config);
        std::vector<feature2d> extract(pic_byte* img, int img_width, int img_height) override;
    private:
        feature_extract_config config_;
        cv::Ptr<cv::FeatureDetector> fastPtr_;
        std::vector<feature2d> features_;
    };
};


#endif // __EXTRACY_FAST_H