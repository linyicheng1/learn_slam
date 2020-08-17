#include "feature_matcher.hpp"
#include "feature_alignment.hpp"

namespace my_slam
{

    bool feature_matcher::findEpipolarMatchDirect(const frame& ref_frame,
                                                  const frame& cur_frame,
                                                  const Eigen::Quaternionf q,
                                                  const Eigen::Vector3f t,
                                                  const feature2d& ref_ftr,
                                                  const float d_estimate,
                                                  const float d_min,
                                                  const float d_max,
                                                  float& depth)
    {
        int zmssd_best = PatchScore::threshold();
        Eigen::Vector2f uv_best;

        // Compute start and end of epipolar line in old_kf for match search, on unit plane!
        const Eigen::Vector3f xyz_min(q.toRotationMatrix()*Eigen::Vector3f(ref_ftr.x_*d_min,ref_ftr.y_*d_min,d_min)+t);
        const Eigen::Vector3f xyz_max(q.toRotationMatrix()*Eigen::Vector3f(ref_ftr.x_*d_max,ref_ftr.y_*d_max,d_max)+t);

        const Eigen::Vector2f A(xyz_min[0]/xyz_min[2],xyz_min[1]/xyz_min[2]);
        const Eigen::Vector2f B(xyz_max[0]/xyz_max[2],xyz_max[1]/xyz_max[2]);
        epi_dir_ = A - B;

        // Compute affine warp matrix
        getWarpMatrixAffine(Eigen::Vector2f(ref_ftr.x_,ref_ftr.y_),d_estimate,q,t,ref_ftr.level_,A_cur_ref_);

//        // feature pre-selection
//        bool reject_ = false;
//        if(ref_ftr.type == Feature::EDGELET && options_.epi_search_edgelet_filtering)
//        {
//            const Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized();
//            const double cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
//            if(cosangle < options_.epi_search_edgelet_max_angle) {
//                reject_ = true;
//                return false;
//            }
//        }

        search_level_ = getBestSearchLevel(A_cur_ref_,cur_frame.pyramid_.size());

        // Find length of search range on epipolar line
        Eigen::Vector2f px_A = cam_->f2c(xyz_min);
        Eigen::Vector2f px_B = cam_->f2c(xyz_max);;
        epi_length_ = (px_A-px_B).norm() / (float)(1<<search_level_);

        // Warp reference patch at ref_level
        warpAffine(A_cur_ref_, ref_frame.pyramid_[ref_ftr.level_], Eigen::Vector2f(ref_ftr.x_,ref_ftr.y_),
                         ref_ftr.level_, search_level_, patch_with_border_);
        createPatchFromPatchWithBorder();

        if(epi_length_ < 2.0)
        {
            px_cur_ = (px_A+px_B)/2.0;
            Eigen::Vector2f px_scaled(px_cur_/(1<<search_level_));
            bool res;
//            if(options_.align_1d)
//                res = align1D(
//                        cur_frame.pyramid_[search_level_], (px_A-px_B).cast<float>().normalized(),
//                        patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
//            else
                res = align2D(cur_frame.pyramid_[search_level_],patch_with_border_,patch_,options_.align_max_iter,px_scaled);
            if(res)
            {
                px_cur_ = px_scaled*(1<<search_level_);
                if(depthFromTriangulation(q,t,cam_->c2f(Eigen::Vector2f(ref_ftr.x_,ref_ftr.y_)),cam_->c2f(px_cur_),depth_))
                    return true;
            }
            return false;
        }

        size_t n_steps = epi_length_/0.7; // one step per pixel
        Eigen::Vector2f step = epi_dir_/n_steps;

        if(n_steps > options_.max_epi_search_steps)
        {
            printf("WARNING: skip epipolar search: %zu evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
                   n_steps, epi_length_, d_min, d_max);
            return false;
        }

        // for matching, precompute sum and sum2 of warped reference patch
        int pixel_sum = 0;
        int pixel_sum_square = 0;
        PatchScore patch_score(patch_);

        // now we sample along the epipolar line
        Eigen::Vector2f uv = B-step;
        Eigen::Vector2i last_checked_pxi(0,0);
        ++n_steps;
        for(size_t i=0; i<n_steps; ++i, uv+=step)
        {
            Eigen::Vector2f px(cam_->c2f(uv).x(),cam_->c2f(uv).y());
            Eigen::Vector2i pxi(px[0]/(1<<search_level_)+0.5,
                         px[1]/(1<<search_level_)+0.5); // +0.5 to round to closest int

            if(pxi == last_checked_pxi)
                continue;
            last_checked_pxi = pxi;

            // TODO interpolation would probably be a good idea
            uint8_t* cur_patch_ptr = cur_frame.pyramid_[search_level_].data
                                     + (pxi[1]-halfpatch_size_)*cur_frame.pyramid_[search_level_].cols
                                     + (pxi[0]-halfpatch_size_);
            int zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.pyramid_[search_level_].cols);

            if(zmssd < zmssd_best) {
                zmssd_best = zmssd;
                uv_best = uv;
            }
        }

        if(zmssd_best < PatchScore::threshold())
        {
            if(options_.subpix_refinement)
            {
                px_cur_ = Eigen::Vector2f(cam_->c2f(uv_best).x(),cam_->c2f(uv_best).y());
                Eigen::Vector2f px_scaled(px_cur_/(1<<search_level_));
                bool res;
//                if(options_.align_1d)
//                    res = feature_alignment::align1D(
//                            cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
//                            patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
//                else
                    res = align2D(cur_frame.pyramid_[search_level_],patch_with_border_,patch_,options_.align_max_iter,px_scaled);
                if(res)
                {
                    px_cur_ = px_scaled*(1<<search_level_);
                    if(depthFromTriangulation(q,t,cam_->c2f(Eigen::Vector2f(ref_ftr.x_,ref_ftr.y_)),cam_->c2f(px_cur_),depth_))
                        return true;
                }
                return false;
            }
            px_cur_ = Eigen::Vector2f(cam_->c2f(uv_best).x(),cam_->c2f(uv_best).y());

            if(depthFromTriangulation(q,t, cam_->c2f(Eigen::Vector2f(ref_ftr.x_,ref_ftr.y_)), cam_->c2f(px_cur_), depth_))
                return true;
        }
        return false;
    }

    feature_matcher::feature_matcher(camera *cam):
    cam_(cam)
    {
    }

    void feature_matcher::getWarpMatrixAffine(const Eigen::Vector2f &px_ref, const double depth_ref,
                                              const Eigen::Quaternionf &q_cur_ref, const Eigen::Vector3f &t_cur_ref,
                                              const int level_ref, Eigen::Matrix2f &A_cur_ref)
    {
        Eigen::Vector3f xyz_kf(px_ref.x()*depth_ref,px_ref.y()*depth_ref,depth_ref);
        Eigen::Vector2f u_kf = px_ref + Eigen::Vector2f(1<<level_ref,0);
        Eigen::Vector2f v_kf = px_ref + Eigen::Vector2f(0,1<<level_ref);

        const Eigen::Vector3f xyz_cur(q_cur_ref.toRotationMatrix()*xyz_kf+t_cur_ref);
        const Eigen::Vector3f u_cur(q_cur_ref.toRotationMatrix()*Eigen::Vector3f(u_kf.x()*depth_ref,u_kf.y()*depth_ref,depth_ref)+t_cur_ref);
        const Eigen::Vector3f v_cur(q_cur_ref.toRotationMatrix()*Eigen::Vector3f(v_kf.x()*depth_ref,v_kf.y()*depth_ref,depth_ref)+t_cur_ref);

        const Eigen::Vector2f uv_cur = cam_->f2c(xyz_cur);
        const Eigen::Vector2f uv_v_cur = cam_->f2c(u_cur);
        const Eigen::Vector2f uv_u_cur = cam_->f2c(v_cur);
        A_cur_ref.col(0) = uv_u_cur-uv_cur;
        A_cur_ref.col(1) = uv_v_cur-uv_cur;
    }
    int feature_matcher::getBestSearchLevel(
            const Eigen::Matrix2f& A_cur_ref,
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
    void feature_matcher::createPatchFromPatchWithBorder()
    {
        uint8_t* ref_patch_ptr = patch_;
        for(int y=1; y<patch_size_+1; ++y, ref_patch_ptr += patch_size_)
        {
            uint8_t* ref_patch_border_ptr = patch_with_border_ + y*(patch_size_+2) + 1;
            for(int x=0; x<patch_size_; ++x)
                ref_patch_ptr[x] = ref_patch_border_ptr[x];
        }
    }

    // https://blog.csdn.net/u011178262/article/details/86729887
    // solve this function [R*P_r P_c][Z_r // Z_c] = t
    bool feature_matcher::depthFromTriangulation(
            const Eigen::Quaternionf& q_search_ref,
            const Eigen::Vector3f& t_search_ref,
            const Eigen::Vector3f& f_ref,
            const Eigen::Vector3f& f_cur,
            float& depth)
    {
        Eigen::Matrix<float,3,2> A;
        A << q_search_ref.toRotationMatrix() * f_ref, f_cur;
        const Eigen::Matrix2f AtA = A.transpose()*A;
        if(AtA.determinant() < 0.000001)
            return false;
        const Eigen::Vector2f depth2 = - AtA.inverse()*A.transpose()*t_search_ref;
        depth = fabs(depth2[0]);
        return true;
    }

    void feature_matcher::warpAffine(const Eigen::Matrix2f &A_cur_ref, const picture &img_ref, const Eigen::Vector2f &px_ref,
                                const int level_ref, const int level_cur, uint8_t *patch)
    {
        const int patch_size = halfpatch_size_*2 ;
        const Eigen::Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
        if(isnan(A_ref_cur(0,0)))
        {
            printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
            return;
        }

        // Perform the warp on a larger patch.
        uint8_t* patch_ptr = patch;
        const Eigen::Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
        for (int y=0; y<patch_size; ++y)
        {
            for (int x=0; x<patch_size; ++x, ++patch_ptr)
            {
                Eigen::Vector2f px_patch(x-halfpatch_size_, y-halfpatch_size_);
                px_patch *= (1<<level_cur);
                const Eigen::Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
                if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
                    *patch_ptr = 0;
                else
                    *patch_ptr = 1;//(uint8_t) interpolateMat_8u(img_ref, px[0], px[1]);
            }
        }
    }

};