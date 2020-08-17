#ifndef __FEATURE_ALIGNMENT_H
#define __FEATURE_ALIGNMENT_H

#include "common.hpp"
#include "eigen3/Eigen/Core"

namespace my_slam {

    bool align1D(
            const picture &cur_img,
            const Eigen::Vector2f& dir,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2d& cur_px_estimate,
            double& h_inv);

    bool align2D(
            const picture &cur_img,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2f& cur_px_estimate,
            bool no_simd = false);

    bool align2D_SSE2(
            const picture &cur_img,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2d& cur_px_estimate);

    bool align2D_NEON(
            const picture &cur_img,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2d& cur_px_estimate);

};
#endif // __FEATURE_ALIGNMENT_H
