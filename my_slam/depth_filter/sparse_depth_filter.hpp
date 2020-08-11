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
        feature2d* ftr;
        float a;
        float b;
        float mu;
        float z_range;
        float sigma2;
        Eigen::Matrix2d patch_cov;
        Seed(feature2d* ftr, float depth_mean, float depth_min);
    };

    class sparse_depth_filter
    {
    public:
        sparse_depth_filter() = default;
        ~sparse_depth_filter() = default;
        void add_frame(picture pic,Eigen::Quaternionf q,Eigen::Vector3f t);
    private:
        bool is_key_frame();

        std::list<Seed, Eigen::aligned_allocator<Seed> > seeds_;
        picture current_frame_;
        Eigen::Quaternionf current_frame_q_;
        Eigen::Vector3f current_frame_t_;

        picture last_kf_;
        Eigen::Quaternionf last_kf_q_;
        Eigen::Vector3f last_kf_t_;
    };

};

#endif // __SPARSE_DEPTH_FILTER_H
