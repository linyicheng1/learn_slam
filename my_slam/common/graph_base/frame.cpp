#include "frame.hpp"
#include <stdint.h>
#include <stdio.h>

namespace my_slam
{
    long int frame::id_ = 0;
    frame::frame(picture img,int levels):
    levels_(levels),
    pic_(img)
    {
        createImgPyramid(pic_,levels_,pyramid_);
    }
    frame::frame(pic_byte* img, int img_width, int img_height,int levels):
    levels_(levels)
    {
        pic_ = picture(img,img_width,img_height);
        createImgPyramid(pic_,levels_,pyramid_);
    }
    /**
    * @brief 创建图像金字塔
    * @param img_level_0 原始图像信息
    * @param n_levels    金字塔层数
    * @param pyr         图像金字塔vector
    */
    void frame::createImgPyramid(picture& img_level_0, int n_levels, ImgPyr& pyr)
    {
        // 设置vector大小
        pyr.resize(n_levels);
        // 第一层就是原始图像
        pyr[0] = img_level_0;
        for(int i=1; i<n_levels; ++i)
        {
            // 每一层都是上一层的降采样，变成原来图片的1/2
            pyr[i].cols = (pyr[i-1].cols - pyr[i-1].cols%2)/2;
            pyr[i].rows = (pyr[i-1].rows - pyr[i-1].rows%2)/2;
            pyr[i].data = new pic_byte[pyr[i].cols*pyr[i].rows];
            half_sample(pyr[i-1],pyr[i]);
        }
    }

    void frame::half_sample(picture src,picture dst)
    {
        const int stride = src.cols;
        pic_byte* top = src.data;
        pic_byte* bottom = top + stride;
        pic_byte* end = top + stride*src.rows;
        const int out_width = dst.cols;
        pic_byte* p = dst.data;
        int tmp = 0;

        while (bottom < end)
        { 
            for (int j=0; j<out_width; j++)
            {
                *p = (pic_byte)((short)top[0] + top[1] + bottom[0] + bottom[1])/4;
                p++;
                top += 2;
                bottom += 2;
                tmp ++;
            }
            
            top += stride;
            bottom += stride;
        }
    }
  
};