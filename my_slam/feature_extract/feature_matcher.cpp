#include "feature_matcher.hpp"

namespace my_slam
{
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
                Eigen::Matrix2d& A_cur_ref)
        {

        }

        int getBestSearchLevel(
                const Eigen::Matrix2d& A_cur_ref,
                const int max_level)
        {
            // Compute patch level in other image
            int search_level = 0;
            double D = A_cur_ref.determinant();
            while(D > 3.0 && search_level < max_level)
            {
                search_level += 1;
                D *= 0.25;
            }
            return search_level;
        }

        void warpAffine(
                const Eigen::Matrix2d& A_cur_ref,
                const cv::Mat& img_ref,
                const Eigen::Vector2d& px_ref,
                const int level_ref,
                const int level_cur,
                const int halfpatch_size,
                uint8_t* patch)
        {
            const int patch_size = halfpatch_size*2 ;
            const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
            if(isnan(A_ref_cur(0,0)))
            {
                printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
                return;
            }

            // Perform the warp on a larger patch.
            uint8_t* patch_ptr = patch;
            const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
            for (int y=0; y<patch_size; ++y)
            {
                for (int x=0; x<patch_size; ++x, ++patch_ptr)
                {
                    Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
                    px_patch *= (1<<search_level);
                    const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
                    if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
                        *patch_ptr = 0;
                    else
                        *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
                }
            }
        }
    };
};

