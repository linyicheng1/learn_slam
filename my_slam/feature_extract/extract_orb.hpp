// #ifndef __ORB_H
// #define __ORB_H

// #include "extract.hpp"
// #include <opencv2/features2d/features2d.hpp>
// #include <vector>

// namespace my_slam
// {
//     class extract_orb:public feature_extract
//     {   
//     public:
//         extract_orb() = default;
//         ~extract_orb() = default;
//         extract_orb(feature_extract_config config);
//         std::vector<feature2d> extract(cv::Mat pic) override;
//     private:
//         feature_extract_config config_;
//         cv::Ptr<cv::FeatureDetector> orbPtr_;
//         std::vector<feature2d> features_;
//     };
// };

// #endif //__ORB_H