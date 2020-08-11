#include "sparse_depth_filter.hpp"

namespace my_slam
{
    /*************************** seed *************************/
    int Seed::batch_counter = 0;
    int Seed::seed_counter = 0;

    Seed::Seed(feature2d *ftr, float depth_mean, float depth_min):
            batch_id(batch_counter),
            id(seed_counter++),
            ftr(ftr),
            a(10),
            b(10),
            mu(1.0f/depth_mean),
            z_range(1.0f/depth_min),
            sigma2(z_range*z_range/36)
    {
    }
    /*************************** filter *************************/

    void sparse_depth_filter::add_frame(picture pic,Eigen::Quaternionf q,Eigen::Vector3f t)
    {
        current_frame_ = pic;
        current_frame_q_ = q;
        current_frame_t_ = t;

        // 当前相机焦距
        const double focal_length = frame->cam_->errorMultiplier2();
        double px_noise = 1.0;
        // 设置当前误差为一个像素点引起的角度误差
        double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0;

        auto it=seeds_.begin();
        while (it!=seeds_.end())
        {
            // check if seed is not already too old
            // 判断当前种子是否太老了
            if((Seed::batch_counter - it->batch_id) > options_.max_n_kfs)
            {// 迭代的次数太多还没有收敛的话就删除吧
                it = seeds_.erase(it);
                continue;
            }

            // check if point is visible in the current image
            // 检测该点是否在当前图像中可以看到
            SE3 T_ref_cur = it->ftr->frame->T_f_w_ * frame->T_f_w_.inverse();
            // 计算当前点的3d位置
            const Vector3d xyz_f(T_ref_cur.inverse()*(1.0/it->mu * it->ftr->f) );
            // 在摄像头后面的就不要
            if(xyz_f.z() < 0.0)
            {
                ++it; // behind the camera
                continue;
            }
            // 不在图像内，也不要
            if(!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>()))
            {
                ++it; // point does not project in image
                continue;
            }

            // we are using inverse depth coordinates
            // 计算最小值
            float z_inv_min = it->mu + sqrt(it->sigma2);
            // 避免到负数，0就是无穷远点
            float z_inv_max = max(it->mu - sqrt(it->sigma2), 0.00000001f);
            double z;

            // 进行极线搜索
            if(!matcher_.findEpipolarMatchDirect(
                    *it->ftr->frame, *frame, *it->ftr, 1.0/it->mu, 1.0/z_inv_min, 1.0/z_inv_max, z))
            {
                // 如果失败的话，记录一下失败次数
                it->b++; // increase outlier probability when no match was found
                ++it;
                ++n_failed_matches;
                continue;
            }

            // 计算 tau
            // compute tau
            double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
            double tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau));

            // update the estimate
            updateSeed(1./z, tau_inverse*tau_inverse, &*it);
            ++n_updates;

            if(frame->isKeyframe())
            {
                // The feature detector should not initialize new seeds close to this location
                feature_detector_->setGridOccpuancy(matcher_.px_cur_);
            }

            // if the seed has converged, we initialize a new candidate point and remove the seed
            if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh)
            {
                assert(it->ftr->point == NULL); // TODO this should not happen anymore
                Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0/it->mu)));
                Point* point = new Point(xyz_world, it->ftr);
                it->ftr->point = point;
                /* FIXME it is not threadsafe to add a feature to the frame here.
                if(frame->isKeyframe())
                {
                  Feature* ftr = new Feature(frame.get(), matcher_.px_cur_, matcher_.search_level_);
                  ftr->point = point;
                  point->addFrameRef(ftr);
                  frame->addFeature(ftr);
                  it->ftr->frame->addFeature(it->ftr);
                }
                else
                */
                {
                    seed_converged_cb_(point, it->sigma2); // put in candidate list
                }
                it = seeds_.erase(it);
            }
            else if(isnan(z_inv_min))
            {
                SVO_WARN_STREAM("z_min is NaN");
                it = seeds_.erase(it);
            }
            else
                ++it;
        }
        if(is_keyframe())
        {
            last_kf_ = pic;
            last_kf_q_ = current_frame_q_;
            last_kf_t_ = current_frame_t_;
        }
    }

    bool sparse_depth_filter::is_key_frame()
    {
        return true;
    }
}
