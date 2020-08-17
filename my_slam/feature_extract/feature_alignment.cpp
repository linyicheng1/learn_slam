#include "feature_alignment.hpp"
#include "eigen3/Eigen/Dense"
#include <math.h>

namespace my_slam
{
    bool align1D(
            const picture &cur_img,
            const Eigen::Vector2f& dir,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2d& cur_px_estimate,
            double& h_inv)
    {

    }

    bool align2D(
            const picture &cur_img,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2f& cur_px_estimate,
            bool no_simd)
    {
#ifdef __ARM_NEON__
        if(!no_simd)
    return align2D_NEON(cur_img, ref_patch_with_border, ref_patch, n_iter, cur_px_estimate);
#endif
        const int halfpatch_size_ = 4;
        const int patch_size_ = 8;
        const int patch_area_ = 64;
        bool converged=false;

       // cmpute derivative of template and prepare inverse compositional
        float __attribute__((__aligned__(16))) ref_patch_dx[patch_area_];
        float __attribute__((__aligned__(16))) ref_patch_dy[patch_area_];
        Eigen::Matrix3f H; H.setZero();

        // compute gradient and hessian
        const int ref_step = patch_size_+2;
        float* it_dx = ref_patch_dx;
        float* it_dy = ref_patch_dy;
        for(int y=0; y<patch_size_; ++y)
        {
            uint8_t* it = ref_patch_with_border + (y+1)*ref_step + 1;
            for(int x=0; x<patch_size_; ++x, ++it, ++it_dx, ++it_dy)
            {
                Eigen::Vector3f J;
                J[0] = 0.5 * (it[1] - it[-1]);
                J[1] = 0.5 * (it[ref_step] - it[-ref_step]);
                J[2] = 1;
                *it_dx = J[0];
                *it_dy = J[1];
                H += J*J.transpose();
            }
        }
        Eigen::Matrix3f Hinv = H.inverse();
        float mean_diff = 0;

        // Compute pixel location in new image:
        float u = cur_px_estimate.x();
        float v = cur_px_estimate.y();

        // termination condition
        const float min_update_squared = 0.03*0.03;
        const int cur_step = cur_img.cols;
        //float chi2 = 0;
        Eigen::Vector3f update; update.setZero();
        for(int iter = 0; iter<n_iter; ++iter)
        {
            int u_r = floor(u);
            int v_r = floor(v);
            if(u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols-halfpatch_size_ || v_r >= cur_img.rows-halfpatch_size_)
                break;

            if(isnan(u) || isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
                return false;

            // compute interpolation weights
            float subpix_x = u-u_r;
            float subpix_y = v-v_r;
            float wTL = (1.0-subpix_x)*(1.0-subpix_y);
            float wTR = subpix_x * (1.0-subpix_y);
            float wBL = (1.0-subpix_x)*subpix_y;
            float wBR = subpix_x * subpix_y;

            // loop through search_patch, interpolate
            uint8_t* it_ref = ref_patch;
            float* it_ref_dx = ref_patch_dx;
            float* it_ref_dy = ref_patch_dy;
//    float new_chi2 = 0.0;
            Eigen::Vector3f Jres; Jres.setZero();
            for(int y=0; y<patch_size_; ++y)
            {
                uint8_t* it = (uint8_t*) cur_img.data + (v_r+y-halfpatch_size_)*cur_step + u_r-halfpatch_size_;
                for(int x=0; x<patch_size_; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy)
                {
                    float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
                    float res = search_pixel - *it_ref + mean_diff;
                    Jres[0] -= res*(*it_ref_dx);
                    Jres[1] -= res*(*it_ref_dy);
                    Jres[2] -= res;
//        new_chi2 += res*res;
                }
            }


/*
    if(iter > 0 && new_chi2 > chi2)
    {
#if SUBPIX_VERBOSE
      cout << "error increased." << endl;
#endif
      u -= update[0];
      v -= update[1];
      break;
    }
    chi2 = new_chi2;
*/
            update = Hinv * Jres;
            u += update[0];
            v += update[1];
            mean_diff += update[2];

#if SUBPIX_VERBOSE
            cout << "Iter " << iter << ":"
         << "\t u=" << u << ", v=" << v
         << "\t update = " << update[0] << ", " << update[1]
//         << "\t new chi2 = " << new_chi2 << endl;
#endif

            if(update[0]*update[0]+update[1]*update[1] < min_update_squared)
            {
#if SUBPIX_VERBOSE
                cout << "converged." << endl;
#endif
                converged=true;
                break;
            }
        }
        cur_px_estimate << u, v;
        return converged;
    }

    bool align2D_SSE2(
            const picture &cur_img,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2d& cur_px_estimate)
    {

    }

    bool align2D_NEON(
            const picture &cur_img,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            Eigen::Vector2d& cur_px_estimate)
    {

    }
}
