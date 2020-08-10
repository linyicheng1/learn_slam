#include "ground_truth.h"

namespace my_slam
{
    ground_truth_EuRoC::ground_truth_EuRoC(std::string path):
    path_(path+"/state_groundtruth_estimate0/data.csv")
    {
        std::ifstream data;
        std::string s;
        data.open(path_.c_str());
        std::getline(data,s);// first line is not the data 
        while(!data.eof())
        {
            // get a line 
            std::getline(data,s);
            if(!s.empty())
            {
                std::stringstream ss(s);
                std::string data_str;
                float data_f;
                Eigen::Vector3f pos;
                float q[4];
                // time stamp 
                std::getline(ss,data_str,',');
                std::stringstream(data_str)>>data_f;
                time_stamp_.push_back(data_f);
                // get first three -> px py pz 
                for(int i=0;i<3;i++)
                {
                    std::getline(ss,data_str,',');
                    std::stringstream(data_str)>>data_f;
                    pos[i] = data_f;
                }
                // get later four -> qw qx qy qz 
                for(int i=0;i<4;i++)
                {
                    std::getline(ss,data_str,',');
                    std::stringstream(data_str)>>data_f;
                    q[i] = data_f;
                }
                pos_.push_back(pos);
                q_.push_back(Eigen::Quaternionf(q[0],q[1],q[2],q[3]));
            }
        }
    }
    ground_truth_depth::ground_truth_depth(std::string path):
    path_(path + "first_200_frames_traj_over_table_input_sequence.txt")
    {
        std::ifstream data;
        std::string s;
        data.open(path_.c_str());
        std::getline(data,s);// first line is not the data
        while(!data.eof())
        {
            if(!s.empty())
            {
                std::stringstream ss(s);
                std::string data_str;
                float data_f;
                Eigen::Vector3f pos;
                float q[4];
                // time stamp
                std::getline(ss,data_str,' ');

                // get first three -> px py pz
                for(int i=0;i<3;i++)
                {
                    std::getline(ss,data_str,' ');
                    std::stringstream(data_str)>>data_f;
                    pos[i] = data_f;
                }
                // get later four -> qw qx qy qz
                for(int i=0;i<4;i++)
                {
                    std::getline(ss,data_str,' ');
                    std::stringstream(data_str)>>data_f;
                    q[i] = data_f;
                }
                pos_.push_back(pos);
                q_.push_back(Eigen::Quaternionf(q[0],q[1],q[2],q[3]));
            }
            // get a line
            std::getline(data,s);
        }
    }

};
