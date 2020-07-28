#ifndef __FRAME_H
#define __FRAME_H
#include "../common.hpp"
#include "feature2d.hpp"

namespace my_slam
{
    class feature2d;

    class frame
    {
    public:    
        frame() =default;
        ~frame() = default;
        frame(pic_byte* img, int img_width, int img_height,int levels);
        frame(picture img,int levels);

        //// 
        void set_key_frame(void){key_frame_ = true;}
        void set_extract_feature(std::vector<feature2d> features){features_ = features;}
        static long int id_;
        ImgPyr pyramid_;
    private:
        bool key_frame_;
        picture pic_;
        int levels_;
        std::vector<feature2d> features_;
        void half_sample(picture src,picture dst);
        void createImgPyramid(picture& img_level_0, int n_levels, ImgPyr& pyr);
    };
};

#endif // __FRAME_H
