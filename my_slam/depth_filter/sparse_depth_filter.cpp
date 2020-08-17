#include "sparse_depth_filter.hpp"
#include <boost/math/distributions/normal.hpp>
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
    void sparse_depth_filter::add_frame(frame *pic)
    {
        if(seeds_.empty())
        {//init
            last_kf_ = new frame(*pic);
            current_frame_ = pic;
            initializeSeeds(last_kf_,10,0.01);
            return;
        }
        current_frame_ = pic;
        size_t n_updates=0, n_failed_matches=0, n_seeds = seeds_.size();
        // 当前相机焦距
        const float focal_length = cam_->get_focal_length();
        float px_noise = 1.0f;
        // 设置当前误差为一个像素点引起的角度误差
        float px_error_angle = atanf(px_noise/(2.0f*focal_length))*2.0f;
        q_cur_ref_ = current_frame_->q_.conjugate()*last_kf_->q_;
        t_cur_ref_ = current_frame_->q_.toRotationMatrix().inverse()*(last_kf_->t_-current_frame_->t_);
        matcher_.pts_A.clear();
        matcher_.pts_B.clear();

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
            // we are using inverse depth coordinates
            // 计算最小值
            float z_inv_min = it->mu + sqrt(it->sigma2);
            // 避免到负数，0就是无穷远点
            float z_inv_max = fmax(it->mu - sqrt(it->sigma2), 0.00000001f);
            float z;
            // 进行极线搜索

            if(!matcher_.findEpipolarMatchDirect(*last_kf_,*current_frame_,q_cur_ref_,t_cur_ref_,it->ftr,1/it->mu,z_inv_min,z_inv_max,z))
            {
                // 如果失败的话，记录一下失败次数
                it->b++; // increase outlier probability when no match was found
                ++it;
                ++n_failed_matches;
                continue;
            }
//
//            // 计算 tau
//            // compute tau
//            float tau = computeTau(cam_->c2f(Eigen::Vector2f(it->ftr.x_,it->ftr.y_)), z, px_error_angle);
//            float tau_inverse = 0.5f * (1.f/fmaxf(0.0000001f, z-tau) - 1.f/(z+tau));
//
//            // update the estimate
//            updateSeed(1.f/z, tau_inverse*tau_inverse, &*it);
//            ++n_updates;

//            if(frame->isKeyframe())
//            {
//                // The feature detector should not initialize new seeds close to this location
//                feature_detector_->setGridOccpuancy(matcher_.px_cur_);
//            }

            // if the seed has converged, we initialize a new candidate point and remove the seed
            if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh)
            {
                //assert(it->ftr->point == NULL); // TODO this should not happen anymore
//                Eigen::Vector3d xyz_world(it->ftr.frame_->T_f_w_.inverse() * (it->ftr->f * (1.0/it->mu)));
//                Point* point = new Point(xyz_world, it->ftr);
//                it->ftr->point = point;
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
//                {
//                    seed_converged_cb_(point, it->sigma2); // put in candidate list
//                }
                auto tmp = point3d(Eigen::Vector3f(it->ftr.x_/it->mu,it->ftr.y_/it->mu,1/it->mu));
                map_.push_back(tmp);
                it = seeds_.erase(it);
            }
            else if(isnan(z_inv_min))
            {

                it = seeds_.erase(it);
            }
            else
                ++it;
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
    void sparse_depth_filter::initializeSeeds(frame *pic,float mean_depth,float min_depth)
    {
        fast_->setExistingFeatures(search_pt_);
        // 提取一些新的特征点
        std::vector<feature2d> points = fast_->extract(pic->pyramid_,pic);

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

        q_cur_ref_ = current_frame_->q_.conjugate() * last_kf_->q_;
        q_cur_ref_.normalize();
        t_cur_ref_ = current_frame_->q_.conjugate().toRotationMatrix()*(last_kf_->t_-current_frame_->t_);

        // 计算当前点的3d位置
        const Eigen::Vector3f xyz(q_cur_ref_.toRotationMatrix()*Eigen::Vector3f(seed.ftr.x_*1.0/seed.mu,seed.ftr.y_*1.0/seed.mu,1.0/seed.mu)+t_cur_ref_);
        // 在摄像头后面的就不要
        if(xyz.z() < 0.0)
        {
            return false;
        }
        const Eigen::Vector2f uv = cam_->f2c(xyz);
        // 不在图像内，也不要
        if(uv[0]>(float)current_frame_->pyramid_[0].cols-6
        ||uv[1]>(float)current_frame_->pyramid_[0].rows-6
        ||uv[0]<6||uv[1]<6)
        {
            return false;
        }
        return true;
    }

    std::list<point3d> sparse_depth_filter::get_depth_map()
    {
        return map_;
    }

    void sparse_depth_filter::updateSeed(const float x, const float tau2, Seed *seed)
    {
        float norm_scale = sqrt(seed->sigma2 + tau2);
        if(std::isnan(norm_scale))
            return;
        boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
        float s2 = 1.f/(1./seed->sigma2 + 1./tau2);
        float m = s2*(seed->mu/seed->sigma2 + x/tau2);
        float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
        float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
        float normalization_constant = C1 + C2;
        C1 /= normalization_constant;
        C2 /= normalization_constant;
        float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
        float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
                  + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));

        // update parameters
        float mu_new = C1*m+C2*seed->mu;
        seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
        seed->mu = mu_new;
        seed->a = (e-f)/(f-e/f);
        seed->b = seed->a*(1.0f-f)/f;
    }

    float sparse_depth_filter::computeTau(const Eigen::Vector3f &f, const float z, const float px_error_angle)
    {
        Eigen::Vector3f a = f*z-t_cur_ref_;
        float t_norm = t_cur_ref_.norm();
        float a_norm = a.norm();
        float alpha = acos(f.dot(t_cur_ref_)/t_norm); // dot product
        float beta = acos(a.dot(-t_cur_ref_)/(t_norm*a_norm)); // dot product
        float beta_plus = beta + px_error_angle;
        float gamma_plus = PI-alpha-beta_plus; // triangle angles sum to PI
        float z_plus = t_norm*sin(beta_plus)/sin(gamma_plus); // law of sines
        return (z_plus - z); // tau
    }
}
