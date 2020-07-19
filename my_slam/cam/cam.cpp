#include "cam.hpp"


namespace my_slam
{
    long long int camera::frame_id_ = 0;
    /************************* EuRoC dataSet *************************/
    EuRoC::EuRoC(std::string pic_path):
    camera(),
    pic_path_(pic_path),
    cam0_path_(pic_path_ + "/cam0/data.csv"),
    cam1_path_(pic_path_ + "/cam1/data.csv"),
    cam_cnt_0_(0),
    cam_cnt_1_(0)
    {
        std::string time_0_path = pic_path_+"/cam0/data";
        std::string time_1_path = pic_path_+"/cam1/data";
        loadImages(time_0_path,cam0_path_,cam0_strImages_,cam0_TimeStamps_);
        loadImages(time_1_path,cam1_path_,cam1_strImages_,cam1_TimeStamps_);
    }
    // default cam0
    cv::Mat EuRoC::getFrame() 
    {
        setUpdateTime(cam0_TimeStamps_[++cam_cnt_0_]);
        updateId();
        return cv::imread(cam0_strImages_[cam_cnt_0_]);
    }
    cv::Mat EuRoC::getFrame2()
    {
        update_time_1_ = cam1_TimeStamps_[++cam_cnt_1_];
        return cv::imread(cam1_strImages_[cam_cnt_1_]);
    }

    void EuRoC::loadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps)
    {
        std::ifstream fTimes;
        std::string s;
        fTimes.open(strPathTimes.c_str());
        vTimeStamps.reserve(5000);
        vstrImages.reserve(5000);
        std::getline(fTimes,s);
        while(!fTimes.eof())
        {
            std::getline(fTimes,s);
            if(!s.empty())
            {
                std::stringstream ss(s);
                std::string time;
                std::getline(ss,time,',');
                vstrImages.push_back(strImagePath + "/" + time + ".png");
                double t;
                std::stringstream(time)>>t;
                vTimeStamps.push_back(t/1e9);
            }
        }
    }

}