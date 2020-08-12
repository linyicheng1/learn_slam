#include "sparse_depth_filter.hpp"

#include <utility>

namespace my_slam
{
    /*************************** seed *************************/
    int Seed::batch_counter = 0;
    int Seed::seed_counter = 0;

    Seed::Seed(feature2d m_ftr, float depth_mean, float depth_min):
            batch_id(batch_counter),
            id(seed_counter++),
            ftr(m_ftr),
            a(10),
            b(10),
            mu(1.0f/depth_mean),
            z_range(1.0f/depth_min),
            sigma2(z_range*z_range/36)
    {
    }
    /*************************** filter *************************/
    sparse_depth_filter::sparse_depth_filter():
        config_(40,30,2,16,0.06)
    {
        fast_ = new extract_fast(config_);
    }
    sparse_depth_filter::sparse_depth_filter(feature_extract_config config):
    config_(config)
    {
        fast_ = new extract_fast(config_);
    }
    void sparse_depth_filter::add_frame(const picture& pic,const Eigen::Quaternionf& q,Eigen::Vector3f t)
    {
        if(seeds_.empty())
        {//init
            initializeSeeds(pic,10,0.01);
            return;
        }
        else
        {
            return;
        }
        current_frame_ = pic;
        current_frame_q_ = q;
        current_frame_t_ = std::move(t);
        size_t n_updates=0, n_failed_matches=0, n_seeds = seeds_.size();
        // 当前相机焦距
        const double focal_length = config_.focal_length_;
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
            // 检测该点是否在当前图像中可以看到
            // check if point is visible in the current image
            if(!is_visible(*it))
            {
                ++it; // behind the camera
                continue;
            }
            // we are using inverse depth coordinates
            // 计算最小值
            float z_inv_min = it->mu + sqrt(it->sigma2);
            // 避免到负数，0就是无穷远点
            float z_inv_max = fmax(it->mu - sqrt(it->sigma2), 0.00000001f);
            double z;

//            // 进行极线搜索
//            if(!matcher_.findEpipolarMatchDirect(current_frame_,last_kf_, *it->ftr, 1.0/it->mu, 1.0/z_inv_min, 1.0/z_inv_max, z))
//            {
//                // 如果失败的话，记录一下失败次数
//                it->b++; // increase outlier probability when no match was found
//                ++it;
//                ++n_failed_matches;
//                continue;
//            }
//
//            // 计算 tau
//            // compute tau
//            double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
//            double tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau));
//
//            // update the estimate
//            updateSeed(1./z, tau_inverse*tau_inverse, &*it);
//            ++n_updates;
//
//            if(frame->isKeyframe())
//            {
//                // The feature detector should not initialize new seeds close to this location
//                feature_detector_->setGridOccpuancy(matcher_.px_cur_);
//            }
//
//            // if the seed has converged, we initialize a new candidate point and remove the seed
//            if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh)
//            {
//                assert(it->ftr->point == NULL); // TODO this should not happen anymore
//                Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0/it->mu)));
//                Point* point = new Point(xyz_world, it->ftr);
//                it->ftr->point = point;
//                /* FIXME it is not threadsafe to add a feature to the frame here.
//                if(frame->isKeyframe())
//                {
//                  Feature* ftr = new Feature(frame.get(), matcher_.px_cur_, matcher_.search_level_);
//                  ftr->point = point;
//                  point->addFrameRef(ftr);
//                  frame->addFeature(ftr);
//                  it->ftr->frame->addFeature(it->ftr);
//                }
//                else
//                */
//                {
//                    seed_converged_cb_(point, it->sigma2); // put in candidate list
//                }
//                it = seeds_.erase(it);
//            }
//            else if(isnan(z_inv_min))
//            {
//
//                it = seeds_.erase(it);
//            }
//            else
//                ++it;
//        }
//        if(is_keyframe())
//        {
//            last_kf_ = pic;
//            last_kf_q_ = current_frame_q_;
//            last_kf_t_ = current_frame_t_;
        }
    }

    bool sparse_depth_filter::is_key_frame()
    {
        return true;
    }

    /**
    * @brief 初始化深度滤波器种子，在新加入一个关键帧时调用
    * @param frame 关键帧
    */
    void sparse_depth_filter::initializeSeeds(const picture& pic,float mean_depth,float min_depth)
    {
        frame_ = new frame(pic.data,pic.cols,pic.rows,config_.levels_);
        fast_->setExistingFeatures(search_pt_);
        // 提取一些新的特征点
        std::vector<feature2d> points = fast_->extract(frame_->pyramid_,frame_);

        ++Seed::batch_counter;
        for(auto pt:points)
        {
            // 特征点构造种子，压入list结构中
            // 初始化当前深度为平均深度，范围为近距离到无穷远
            seeds_.emplace_back(Seed(pt, mean_depth, min_depth));
        }
    }

    bool sparse_depth_filter::is_visible(Seed seed)
    {
        Eigen::Quaternionf q_cur_ref;
        Eigen::Vector3f t_cur_ref;
        q_cur_ref = current_frame_q_.inverse() * last_kf_q_;
        q_cur_ref.normalize();
        t_cur_ref = current_frame_q_.inverse().toRotationMatrix()*(last_kf_t_-current_frame_t_);

        // 计算当前点的3d位置
        const Eigen::Vector3f xyz_f(current_frame_q_.toRotationMatrix()*Eigen::Vector3f(seed.ftr.x_*1.0/seed.mu,seed.ftr.y_*1.0/seed.mu,1.0/seed.mu)+t_cur_ref);
        // 在摄像头后面的就不要
        if(xyz_f.z() < 0.0)
        {
            return false;
        }
        int u = (int)(config_.focal_length_*xyz_f[0]/xyz_f[2]);
        int v = (int)(config_.focal_length_*xyz_f[1]/xyz_f[2]);
        // 不在图像内，也不要
        if(u>current_frame_.cols/2||v>current_frame_.rows/2)
        {
            return false;
        }
        return true;
    }

    std::vector<Eigen::Vector3f> sparse_depth_filter::get_depth_filter()
    {
        std::vector<Eigen::Vector3f> depth_map;
        for(const auto& seed:seeds_)
        {
            float z = 1.f/seed.mu;
            Eigen::Vector3f xyz_f = cam_->c2f(Eigen::Vector2f(seed.ftr.x_,seed.ftr.y_));
            depth_map.emplace_back(xyz_f*z);
        }
        return depth_map;
    }
}
