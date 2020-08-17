#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

#include "eigen3/Eigen/Core"
#include "../cam/cam.hpp"
#include "eigen3/Eigen/Geometry"
#include "graph_base/feature2d.hpp"
#include "graph_base/frame.hpp"
#include "feature_alignment.hpp"
#include "patch_score.hpp"
#include <vector>

namespace my_slam
{

    class feature_matcher
    {
        static const int halfpatch_size_ = 4;
        static const int patch_size_ = 8;
        typedef patch_score::ZMSSD<halfpatch_size_> PatchScore;
    public:
        struct Options
        {
            bool align_1d;              //!< in epipolar search: align patch 1D along epipolar line
            int align_max_iter;         //!< number of iterations for aligning the feature patches in gauss newton
            double max_epi_length_optim;//!< max length of epipolar line to skip epipolar search and directly go to img align
            size_t max_epi_search_steps;//!< max number of evaluations along epipolar line
            bool subpix_refinement;     //!< do gauss newton feature patch alignment after epipolar search
            bool epi_search_edgelet_filtering;
            double epi_search_edgelet_max_angle;
            Options() :
                    align_1d(false),
                    align_max_iter(10),
                    max_epi_length_optim(2.0),
                    max_epi_search_steps(1000),
                    subpix_refinement(true),
                    epi_search_edgelet_filtering(true),
                    epi_search_edgelet_max_angle(0.7)
            {}
        } options_;

        feature_matcher() = default;
        feature_matcher(camera* cam);
        ~feature_matcher() = default;
        bool findEpipolarMatchDirect(
                const frame& ref_frame,
                const frame& cur_frame,
                const Eigen::Quaternionf q,
                const Eigen::Vector3f t,
                const feature2d& ref_ftr,
                const float d_estimate,
                const float d_min,
                const float d_max,
                float& depth);
        void set_cam(camera *cam){cam_=cam;}
        /////// for view //////
        std::vector<Eigen::Vector2f> pts_A;
        std::vector<Eigen::Vector2f> pts_B;
    private:
        Eigen::Vector2f epi_dir_;
        Eigen::Vector2f px_cur_;
        camera *cam_;
        Eigen::Matrix2f A_cur_ref_;
        int search_level_;
        float epi_length_;
        float depth_;
        float h_inv_;

        uint8_t patch_[patch_size_*patch_size_] __attribute__ ((aligned (16)));
        uint8_t patch_with_border_[(patch_size_+2)*(patch_size_+2)] __attribute__ ((aligned (16)));

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
        void createPatchFromPatchWithBorder();
        bool depthFromTriangulation(
                const Eigen::Quaternionf& q_search_ref,
                const Eigen::Vector3f& t_search_ref,
                const Eigen::Vector3f& f_ref,
                const Eigen::Vector3f& f_cur,
                float& depth);
        void warpAffine(
                const Eigen::Matrix2f& A_cur_ref,
                const picture& img_ref,
                const Eigen::Vector2f& px_ref,
                const int level_ref,
                const int level_cur,
                uint8_t* patch);
    };
};
#endif // __FEATURE_MATCHER_H
