#ifndef __SPARSE_DEPTH_FILTER_H
#define __SPARSE_DEPTH_FILTER_H
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <opencv2/opencv.hpp>
#include "../cam/cam.hpp"
#include <queue>
#include "eigen3/Eigen/StdList"
#include <boost/thread.hpp>
#include <boost/function.hpp>

namespace my_slam
{
    struct Seed
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static int batch_counter;
        static int seed_counter;
        int batch_id;                //!< Batch id is the id of the keyframe for which the seed was created.
        int id;                      //!< Seed ID, only used for visualization.
        float a;                     //!< a of Beta distribution: When high, probability of inlier is large.
        float b;                     //!< b of Beta distribution: When high, probability of outlier is large.
        float mu;                    //!< Mean of normal distribution.
        float z_range;               //!< Max range of the possible depth.
        float sigma2;                //!< Variance of normal distribution.
        Eigen::Matrix2d patch_cov;          //!< Patch covariance in reference image.
        Seed(float depth_mean, float depth_min);
    };

    class sparse_depth_filter
    {
    public:

        sparse_depth_filter(camera *cam);
        void init();
        ~sparse_depth_filter();

        void addFrame(cv::Mat frame);
        void addKeyframe(cv::Mat frame, double depth_mean, double depth_min);

        void pose_update(){seeds_updating_halt_ = true;}
        void restart(){seeds_updating_halt_ = false;}
    private:
        void update_seeds(cv::Mat frame);
        void init_seeds();
        std::list<Seed, Eigen::aligned_allocator<Seed> > seeds_;

        camera *cam_;
        bool seeds_updating_halt_ = false;
        boost::mutex seeds_mut_;
    };
};

#endif // __SPARSE_DEPTH_FILTER_H
