#ifndef __POINT3D_H
#define __POINT3D_H

#include <utility>

#include "eigen3/Eigen/Core"

class point3d{
public:
    point3d() = default;
    point3d(Eigen::Vector3f pos):
    pos_(std::move(pos))
    {
    }
    ~point3d() = default;
    void set_pos(const Eigen::Vector3f pos){pos_ = pos;}
    const Eigen::Vector3f get_pos(){return pos_;}
private:
    Eigen::Vector3f pos_;
};

#endif // __POINT3D_H
