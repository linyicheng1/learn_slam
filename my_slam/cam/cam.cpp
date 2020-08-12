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
    /************************* depth dataSet *************************/

    depth_data::depth_data(std::string pic_path):
    camera(),
    pic_path_(pic_path),
    cam_path_(pic_path + "first_200_frames_traj_over_table_input_sequence.txt"),
    cam_cnt_(0)
    {
        loadImages(cam_path_,pic_path_,cam_strImages_);
        // get inner param
        cam_param.fx = 481.2f;
        cam_param.fy = -480.0f;
        cam_param.u0 = 319.5f;
        cam_param.v0 = 239.5f;
    }

    void depth_data::loadImages(const std::string &strImagePath,const std::string &strPathTimes,std::vector<std::string> &vstrImages)
    {
        std::ifstream fTimes;
        std::string s;
        fTimes.open(strImagePath.c_str());
        vstrImages.reserve(5000);
        std::getline(fTimes,s);
        while(!fTimes.eof())
        {

            if(!s.empty())
            {
                std::stringstream ss(s);
                std::string time;
                std::getline(ss,time,' ');
                vstrImages.push_back(strPathTimes + "images/" + time);
            }
            std::getline(fTimes,s);
        }
    }

    cv::Mat depth_data::getFrame()
    {
        auto tmp = cv::imread(cam_strImages_.at(getFrameId()));
        updateId();
        return tmp;
    }
}