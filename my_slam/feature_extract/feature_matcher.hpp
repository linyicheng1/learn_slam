#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

#include "eigen3/Eigen/Core"
#include "../cam/cam.hpp"
#include "eigen3/Eigen/Geometry"
#include "graph_base/feature2d.hpp"
#include "graph_base/frame.hpp"

namespace my_slam
{
    class feature_matcher
    {
    public:
        feature_matcher() = default;
        feature_matcher(camera* cam);
        ~feature_matcher() = default;
        bool findEpipolarMatchDirect(
                const frame& ref_frame,
                const frame& cur_frame,
                const Eigen::Quaternionf q,
                const Eigen::Vector3f t,
                const feature2d& ref_ftr,
                const double d_estimate,
                const double d_min,
                const double d_max,
                double& depth);

    private:
        Eigen::Vector2f epi_dir_;
        camera *cam_;
        Eigen::Matrix2f A_cur_ref_;
        int search_level_;
        float epi_length_;
        void getWarpMatrixAffine(
                const Eigen::Vector2f& px_ref,
                const double depth_ref,
                const Eigen::Quaternionf& q_cur_ref,
                const Eigen::Vector3f& t_cur_ref,
                const int level_ref,
                Eigen::Matrix2f& A_cur_ref);
        int getBestSearchLevel(
                const Eigen::Matrix2f& A_cur_ref,
                const int max_level);
    };
};
#endif // __FEATURE_MATCHER_H
