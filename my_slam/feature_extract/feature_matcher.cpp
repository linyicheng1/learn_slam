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
//            // Compute patch level in other image
//            int search_level = 0;
//            double D = A_cur_ref.determinant();
//            while(D > 3.0 && search_level < max_level)
//            {
//                search_level += 1;
//                D *= 0.25;
//            }
//            return search_level;
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
//            const int patch_size = halfpatch_size*2 ;
//            const Eigen::Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
//            if(isnan(A_ref_cur(0,0)))
//            {
//                printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
//                return;
//            }
//
//            // Perform the warp on a larger patch.
//            uint8_t* patch_ptr = patch;
//            const Eigen::Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
//            for (int y=0; y<patch_size; ++y)
//            {
//                for (int x=0; x<patch_size; ++x, ++patch_ptr)
//                {
//                    Eigen::Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
//                    px_patch *= (1<<search_level);
//                    const Eigen::Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
//                    if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
//                        *patch_ptr = 0;
//                    else
//                        *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
//                }
//            }
        }
    };

    bool feature_matcher::findEpipolarMatchDirect(const picture& ref_frame,
                                                  const picture& cur_frame,
                                                  const Eigen::Quaternionf q,
                                                  const Eigen::Vector3f t,
                                                  const Eigen::Vector3f & ref_ftr,
                                                  const double d_estimate,
                                                  const double d_min,
                                                  const double d_max,
                                                  double& depth)
    {
//        //int zmssd_best = PatchScore::threshold();
//        Eigen::Vector2d uv_best;
//
//        // Compute start and end of epipolar line in old_kf for match search, on unit plane!
//        Eigen::Vector2f A = project2d(q.toRotationMatrix() * (ref_ftr*d_min) + t);
//        Eigen::Vector2f B = project2d(q.toRotationMatrix() * (ref_ftr*d_max) + t);
//        epi_dir_ = A - B;
//
//        // Compute affine warp matrix
//        warp::getWarpMatrixAffine(
//                *ref_frame.cam_, *cur_frame.cam_, ref_ftr.px, ref_ftr.f,
//                d_estimate, T_cur_ref, ref_ftr.level, A_cur_ref_);
//
//        // feature pre-selection
//        reject_ = false;
//        if(ref_ftr.type == Feature::EDGELET && options_.epi_search_edgelet_filtering)
//        {
//            const Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized();
//            const double cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
//            if(cosangle < options_.epi_search_edgelet_max_angle) {
//                reject_ = true;
//                return false;
//            }
//        }
//
//        search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);
//
//        // Find length of search range on epipolar line
//        Vector2d px_A(cur_frame.cam_->world2cam(A));
//        Vector2d px_B(cur_frame.cam_->world2cam(B));
//        epi_length_ = (px_A-px_B).norm() / (1<<search_level_);
//
//        // Warp reference patch at ref_level
//        warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
//                         ref_ftr.level, search_level_, halfpatch_size_+1, patch_with_border_);
//        createPatchFromPatchWithBorder();
//
//        if(epi_length_ < 2.0)
//        {
//            px_cur_ = (px_A+px_B)/2.0;
//            Vector2d px_scaled(px_cur_/(1<<search_level_));
//            bool res;
//            if(options_.align_1d)
//                res = feature_alignment::align1D(
//                        cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
//                        patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
//            else
//                res = feature_alignment::align2D(
//                        cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
//                        options_.align_max_iter, px_scaled);
//            if(res)
//            {
//                px_cur_ = px_scaled*(1<<search_level_);
//                if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
//                    return true;
//            }
//            return false;
//        }
//
//        size_t n_steps = epi_length_/0.7; // one step per pixel
//        Vector2d step = epi_dir_/n_steps;
//
//        if(n_steps > options_.max_epi_search_steps)
//        {
//            printf("WARNING: skip epipolar search: %zu evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
//                   n_steps, epi_length_, d_min, d_max);
//            return false;
//        }
//
//        // for matching, precompute sum and sum2 of warped reference patch
//        int pixel_sum = 0;
//        int pixel_sum_square = 0;
//        PatchScore patch_score(patch_);
//
//        // now we sample along the epipolar line
//        Vector2d uv = B-step;
//        Vector2i last_checked_pxi(0,0);
//        ++n_steps;
//        for(size_t i=0; i<n_steps; ++i, uv+=step)
//        {
//            Vector2d px(cur_frame.cam_->world2cam(uv));
//            Vector2i pxi(px[0]/(1<<search_level_)+0.5,
//                         px[1]/(1<<search_level_)+0.5); // +0.5 to round to closest int
//
//            if(pxi == last_checked_pxi)
//                continue;
//            last_checked_pxi = pxi;
//
//            // check if the patch is full within the new frame
//            if(!cur_frame.cam_->isInFrame(pxi, patch_size_, search_level_))
//                continue;
//
//            // TODO interpolation would probably be a good idea
//            uint8_t* cur_patch_ptr = cur_frame.img_pyr_[search_level_].data
//                                     + (pxi[1]-halfpatch_size_)*cur_frame.img_pyr_[search_level_].cols
//                                     + (pxi[0]-halfpatch_size_);
//            int zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.img_pyr_[search_level_].cols);
//
//            if(zmssd < zmssd_best) {
//                zmssd_best = zmssd;
//                uv_best = uv;
//            }
//        }
//
//        if(zmssd_best < PatchScore::threshold())
//        {
//            if(options_.subpix_refinement)
//            {
//                px_cur_ = cur_frame.cam_->world2cam(uv_best);
//                Vector2d px_scaled(px_cur_/(1<<search_level_));
//                bool res;
//                if(options_.align_1d)
//                    res = feature_alignment::align1D(
//                            cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
//                            patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
//                else
//                    res = feature_alignment::align2D(
//                            cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
//                            options_.align_max_iter, px_scaled);
//                if(res)
//                {
//                    px_cur_ = px_scaled*(1<<search_level_);
//                    if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
//                        return true;
//                }
//                return false;
//            }
//            px_cur_ = cur_frame.cam_->world2cam(uv_best);
//            if(depthFromTriangulation(T_cur_ref, ref_ftr.f, vk::unproject2d(uv_best).normalized(), depth))
//                return true;
//        }
        return false;
    }
    Eigen::Vector2f feature_matcher::project2d(Eigen::Vector3f pt_f)
    {

    }
};

