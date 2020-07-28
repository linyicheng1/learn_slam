// #include "extract_orb.hpp"

// namespace my_slam
// {

//     std::vector<feature2d> extract_orb::extract(cv::Mat pic)
//     {   
//         std::cout<<"extract!"<<std::endl;
//         std::vector<cv::KeyPoint> keyPoints;
//         orbPtr_->detect(pic, keyPoints);
//         features_.empty();
//         features_.resize(keyPoints.size());
//         for(auto it:keyPoints)
//         {
//             feature2d feature;
//             feature.set_feature((int)it.pt.x,(int)it.pt.y);
//             features_.push_back(feature);
//         }
//         return features_;
//     }
//     extract_orb::extract_orb(feature_extract_config config)
//     {
//         config_ = config;
//         orbPtr_ = cv::ORB::create(1000, 1.2, 8, 31, 0, 2, 0, 31, 20);
//         features_.resize(10);
//     }           
// };
