#include "orb.hpp"
#include <vector>

int main()
{
    auto raw_1 = cv::imread("../1.png");
    auto raw_2 = cv::imread("../2.png");
    /****************采用opencv实现****************/
    // 定义变量
    cv::Ptr<cv::ORB> orb_1 = cv::ORB::create(1000, 1.2, 8, 31, 0, 2, 0, 31, 20);
    cv::Ptr<cv::ORB> orb_2 = cv::ORB::create(1000, 1.2, 8, 31, 0, 2, 0, 31, 20);
    std::vector<cv::KeyPoint> kps_1,kps_2;
    cv::Mat desc_1,desc_2;
    std::vector<cv::DMatch> matches,good_matches;
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    cv::Mat feature_pic_1,feature_pic_2,match_pic,good_match_pic;
    // 计算特征点和描述子
    orb_1->detect(raw_1,kps_1);
    orb_1->compute(raw_1,kps_1,desc_1);
    orb_2->detect(raw_2,kps_2);
    orb_2->compute(raw_2,kps_2,desc_2);
    // 对两张图片进行匹配
    matcher->match ( desc_1, desc_2, matches );
    for(auto match:matches)
    {
        if(match.distance<40)
        {
            good_matches.push_back(match);
        }
    }
    // 输出图片
    cv::drawKeypoints(raw_1,kps_1,feature_pic_1);
    cv::drawKeypoints(raw_2,kps_2,feature_pic_2);
    cv::drawMatches(raw_1,kps_1,raw_2,kps_2,matches,match_pic);
    cv::drawMatches(raw_1,kps_1,raw_2,kps_2,good_matches,good_match_pic);
    cv::imshow("feature1",feature_pic_1);
    cv::imshow("feature2",feature_pic_2);
    cv::imshow("matches",match_pic);
    cv::imshow("good matches",good_match_pic);
    cv::waitKey();
}
