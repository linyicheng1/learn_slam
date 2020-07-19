#ifndef __CAMERA__H
#define __CAMERA__H

#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <iostream>

namespace my_slam
{
    enum type
    {
        CAMERA = 1,
        PIC,
        VIDEO
    };
    // 相机，纯虚类
    class camera
    {
    public:
        // default function
        camera() = default;
        ~camera() = default;
        // interface 
        virtual cv::Mat getFrame() = 0;
        virtual int getType() = 0;
        virtual inline double getUpdateTime(){ return update_time_;}
        virtual inline int getFrameId(){ return frame_id_;}
        inline void updateId(){frame_id_ ++;}
        inline void setUpdateTime(double time){update_time_ = time;}

        cv::Mat frame_;
    private:
        static long long int frame_id_;
        double update_time_;
    };

    // 由虚类派生成四个相机类代表相机、图片数据和视频数据
    // class cam:public camera
    // {
    // public:
    //     cam() = default;
    //     cv::Mat getFrame() override;
    //     int getType() override{ return CAMERA;}
    // private:
    // };
    // class video:public camera
    // {
    // public:
    //     video() = default;
    //     explicit video(const std::string& video_path);
    //     cv::Mat getFrame() override;
    //     int getType() override{ return VIDEO;}
    // private:
    //     cv::VideoCapture cap_;
    // };

    // euroc dataSet
    class EuRoC:public camera
    {
    public:
        EuRoC() = default;
        ~EuRoC() = default;
        EuRoC(std::string pic_path);
        cv::Mat getFrame() override;
        cv::Mat getFrame2();
        int getType() override{ return PIC;}
    private:
        std::string pic_path_;
        std::string cam0_path_;
        std::string cam1_path_;

        std::vector<std::string> cam0_strImages_;
        std::vector<double> cam0_TimeStamps_;
        int cam_cnt_0_;

        std::vector<std::string> cam1_strImages_;
        std::vector<double> cam1_TimeStamps_;
        int cam_cnt_1_;
        double update_time_1_;

        void loadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps);
    };
}




#endif //__CAMERA__H
