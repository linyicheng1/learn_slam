#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

#include "eigen3/Eigen/Core"
#include "../cam/cam.hpp"
#include "eigen3/Eigen/Geometry"

namespace my_slam
{
    /// Warp a patch from the reference view to the current view.
    namespace warp
    {
        void getWarpMatrixAffine(
                const camera& cam_ref,
                const camera& cam_cur,
                const Eigen::Vector2d& px_ref,
                const Eigen::Vector3d& f_ref,
                const double depth_ref,
                const Eigen::Quaternionf& q_cur_ref,
                const Eigen::Vector3f& t_cur_ref,
                const int level_ref,
                Eigen::Matrix2d& A_cur_ref);

        int getBestSearchLevel(
                const Eigen::Matrix2d& A_cur_ref,
                const int max_level);

        void warpAffine(
                const Eigen::Matrix2d& A_cur_ref,
                const cv::Mat& img_ref,
                const Eigen::Vector2d& px_ref,
                const int level_ref,
                const int level_cur,
                const int halfpatch_size,
                uint8_t* patch);
    } // namespace warp

    class matcher
    {

    };
};
#endif // __FEATURE_MATCHER_H
