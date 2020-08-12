#ifndef __SPARSE_DEPTH_FILTER_H
#define __SPARSE_DEPTH_FILTER_H
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>
#include <vector>
#include "../cam/cam.hpp"
#include <queue>
#include "eigen3/Eigen/StdList"
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include "common.hpp"
#include "graph_base/feature2d.hpp"
#include "../feature_extract/extract.hpp"
#include "../feature_extract/extract_fast.hpp"
#include "../feature_extract/feature_matcher.hpp"

namespace my_slam
{
/// A seed is a probabilistic depth estimate for a single pixel.
    struct Seed
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static int batch_counter;
        static int seed_counter;
        int batch_id;
        int id;
        feature2d ftr;
        float a;
        float b;
        float mu;
        float z_range;
        float sigma2;
        Eigen::Matrix2d patch_cov;
        Seed(feature2d m_ftr, float depth_mean, float depth_min);
    };

    class sparse_depth_filter
    {
    private:
        /// Depth-filter config parameters
        struct Options
        {
            bool check_ftr_angle;                       //!< gradient features are only updated if the epipolar line is orthogonal to the gradient.
            bool epi_search_1d;                         //!< restrict Gauss Newton in the epipolar search to the epipolar line.
            bool verbose;                               //!< display output.
            bool use_photometric_disparity_error;       //!< use photometric disparity error instead of 1px error in tau computation.
            int max_n_kfs;                              //!< maximum number of keyframes for which we maintain seeds.
            double sigma_i_sq;                          //!< image noise.
            double seed_convergence_sigma2_thresh;      //!< threshold on depth uncertainty for convergence.
            Options():
            check_ftr_angle(false),
            epi_search_1d(false),
            verbose(false),
            use_photometric_disparity_error(false),
            max_n_kfs(5),
            sigma_i_sq(5e-4),
            seed_convergence_sigma2_thresh(200.0)
            {}
        }options_;
    public:
        sparse_depth_filter();
        explicit sparse_depth_filter(feature_extract_config config);
        ~sparse_depth_filter() = default;
        void set_cam(camera* cam){cam_ = cam;}
        void add_frame(const picture& pic,const Eigen::Quaternionf& q,Eigen::Vector3f t);
        std::vector<Eigen::Vector3f> get_depth_filter();
    private:
        bool is_key_frame();
        bool is_visible(Seed seed);
        void initializeSeeds(const picture& pic,float mean_depth,float min_depth);

        feature_extract_config  config_;
        extract_fast *fast_;
        camera* cam_;
        std::list<Seed> seeds_;
        frame *frame_;
        std::vector<feature2d> search_pt_;
        feature_matcher matcher_;

        picture current_frame_;
        Eigen::Quaternionf current_frame_q_;
        Eigen::Vector3f current_frame_t_;

        picture last_kf_;
        Eigen::Quaternionf last_kf_q_;
        Eigen::Vector3f last_kf_t_;

        Eigen::Quaternionf q_cur_ref_;
        Eigen::Vector3f t_cur_ref_;
    };

};

#endif // __SPARSE_DEPTH_FILTER_H
