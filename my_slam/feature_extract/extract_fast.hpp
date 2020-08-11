#ifndef __EXTRACY_FAST_H
#define __EXTRACY_FAST_H

#include "extract.hpp"
#include <vector>
#include "../thirdPart_lib/fast/include/fast/fast.h"
#include "common.hpp"

//#define USE_OPENCV

#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
#endif

namespace my_slam
{
    class feature2d;
    struct Corner
    {
        int x;        //!< x-coordinate of corner in the image.
        int y;        //!< y-coordinate of corner in the image.
        int level;    //!< pyramid level of the corner.
        float score;  //!< shi-tomasi score of the corner.
        float angle;  //!< for gradient-features: dominant gradient angle.
        Corner(int x, int y, float score, int level, float angle) :
        x(x), y(y), level(level), score(score), angle(angle)
        {}
    };
    typedef std::vector<Corner> Corners;
    
    class extract_fast:public feature_extract
    {   
    public:
        extract_fast() = default;
        ~extract_fast() = default;
        extract_fast(feature_extract_config config);
        std::vector<feature2d> extract(pic_byte* img, int img_width, int img_height,int level ,frame* Frame) override;
        std::vector<feature2d> extract(ImgPyr pyramid,frame* Frame) override;
        void resetGrid();
    private:
        feature_extract_config config_;
        std::vector<feature2d> features_;
        std::vector<bool> grid_occupancy_;
        int threshold_;
        Corners* corners_;
        float shiTomasiScore(picture img,int u,int v);
    };
};


#endif // __EXTRACY_FAST_H