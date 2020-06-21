// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <stdint.h>

#include <unordered_set>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include<suitesparse/cholmod.h>
using namespace Eigen;
using namespace std;

class Sample {
 public:
  static int uniform(int from, int to) { return static_cast<int>(g2o::Sampler::uniformRand(from, to)); }
};

// 相对于源码去除掉了参数输入，从而方便调试
int main(int argc, const char* argv[])
{
  // 像素噪声
  double PIXEL_NOISE = 0.01;
  // 离群点半径
  double OUTLIER_RATIO = 0.1;
  // 是否使用鲁棒核函数
  bool ROBUST_KERNEL = true;
  // 是否稠密
  bool DENSE = false;
  /********************初始化求解器*********************/
  // 定义求解器
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  // BA线性求解BlockSolver_6_3,含义为节点维度为6 边的维度为3
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
  // 选择不同的求解器，如果结构是稀疏的则选择稀疏求解器，稠密结构则选择稠密求求解
  // 合理的利用H矩阵的稀疏性能够极大提高求解速度
  if (DENSE) {
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
  } else {
    linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
  }
  // 选择迭代策略，通常还是L-M算法居多
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
  );
  optimizer.setAlgorithm(solver);

  /********************构造图结构*********************/
  vector<Vector3d> true_points;//存储真实值
  for (size_t i=0;i<500; ++i)
  {
    // 随机产生500个真实点
    // 范围为 x:-1.5～1.5 y:-0.5~0.5 z:3~4
    true_points.push_back(Vector3d((g2o::Sampler::uniformRand(0., 1.)-0.5)*3,
                                   g2o::Sampler::uniformRand(0., 1.)-0.5,
                                   g2o::Sampler::uniformRand(0., 1.)+3));
  }
  // 这里虚拟了一个相机出来
  // 焦距为1000（单位？）
  double focal_length= 1000.;
  // 中心点为320,240.图片大小即为640×480
  Vector2d principal_point(320., 240.);
  // 真实的位姿记录
  vector<g2o::SE3Quat,
      aligned_allocator<g2o::SE3Quat> > true_poses;
  // 虚拟相机，给定参数
  g2o::CameraParameters * cam_params
      = new g2o::CameraParameters (focal_length, principal_point, 0.);
  cam_params->setId(0);
  // 将相机的参数添加到优化器中
  if (!optimizer.addParameter(cam_params)) {
    assert(false);
  }
  // 计算虚拟15帧的数据
  int vertex_id = 0;
  for (size_t i=0; i<15; ++i) {
    // x轴进行缓慢的移动，旋转始终保持正视
    Vector3d trans(i*0.04-1.,0,0);
    Eigen:: Quaterniond q;
    q.setIdentity();
    // 得到对应的姿态
    g2o::SE3Quat pose(q,trans);
    // se3 的节点
    g2o::VertexSE3Expmap * v_se3
        = new g2o::VertexSE3Expmap();
    v_se3->setId(vertex_id);
    // 前两帧id的位姿是固定的，在优化过程中不变
    if (i<2){
      v_se3->setFixed(true);
    }
    // 将需要优化的位姿信息添加到图结构中
    v_se3->setEstimate(pose);
    optimizer.addVertex(v_se3);
    true_poses.push_back(pose);
    vertex_id++;
  }
  // 节点的id在帧id基础上加
  int point_id=vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;
  cout << endl;
  unordered_map<int,int> pointid_2_trueid;
  unordered_set<int> inliers;

  // 遍历所有3D点
  for (size_t i=0; i<true_points.size(); ++i)
  {
    // 3D点节点
    g2o::VertexSBAPointXYZ * v_p
        = new g2o::VertexSBAPointXYZ();
    // 设置id
    v_p->setId(point_id);
    // 设置为边缘化，和姿态节点区分便于稀疏求解
    v_p->setMarginalized(true);
    // 初始值+噪声
    v_p->setEstimate(true_points.at(i)
                     + Vector3d(g2o::Sampler::gaussRand(0., 1),
                                g2o::Sampler::gaussRand(0., 1),
                                g2o::Sampler::gaussRand(0., 1)));
    // 计算被观测到的点
    int num_obs = 0;
    // 遍历所有的位姿
    for (size_t j=0; j<true_poses.size(); ++j)
    {
      // 计算当前点在j帧相机下投影位置
      Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
      // 如果在图片范围内，则认为该点能够被看到
      if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
        ++num_obs;
      }
    }
    // >=2 个点能看到就能够计算地图点的位置
    if (num_obs>=2)
    {
      // 将地图点添加到优化中
      optimizer.addVertex(v_p);
      bool inlier = true;
      // 遍历所有关键帧
      for (size_t j=0; j<true_poses.size(); ++j)
      {
        // 计算当前点在当前帧下的像素坐标
        Vector2d z
            = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
        // 如果在当前帧下能够看到该点
        if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480)
        {
          // 随机抽取离群点
          double sam = g2o::Sampler::uniformRand(0., 1.);
          if (sam<OUTLIER_RATIO)
          {// 10%的离群点的位置是随机的
            z = Vector2d(Sample::uniform(0,640),
                         Sample::uniform(0,480));
            inlier= false;
          }
          // 非离群点则只需要加上一个高斯噪声
          z += Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                        g2o::Sampler::gaussRand(0., PIXEL_NOISE));
          // 约束边
          g2o::EdgeProjectXYZ2UV * e
              = new g2o::EdgeProjectXYZ2UV();
          // 设置关联的3D地图点和当前帧
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                       (optimizer.vertices().find(j)->second));
          e->setMeasurement(z);
          e->information() = Matrix2d::Identity();
          // 是否使用鲁棒核，使用huber鲁棒核能有效抑制外点干扰
          if (ROBUST_KERNEL) {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
          }
          // 添加到优化器中
          e->setParameterId(0, 0);
          optimizer.addEdge(e);
        }
      }
      // 计算一下初始内点的误差，并记录一下哪些是内点
      if (inlier)
      {
        inliers.insert(point_id);
        Vector3d diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      pointid_2_trueid.insert(make_pair(point_id,i));
      ++point_id;
      ++point_num;
    }
  }
  cout << endl;
  /********************求解问题*********************/
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  //optimizer.save("test.g2o");
  cout << endl;
  cout << "Performing full BA:" << endl;
  optimizer.optimize(10);
  cout << endl;
  cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;
  point_num = 0;
  sum_diff2 = 0;
  // 计算优化后的内点误差
  for (unordered_map<int,int>::iterator it=pointid_2_trueid.begin();
       it!=pointid_2_trueid.end(); ++it)
  {
    g2o::HyperGraph::VertexIDMap::iterator v_it
        = optimizer.vertices().find(it->first);
    if (v_it==optimizer.vertices().end()){
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }
    g2o::VertexSBAPointXYZ * v_p
        = dynamic_cast< g2o::VertexSBAPointXYZ * > (v_it->second);
    if (v_p==0){
      cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
      exit(-1);
    }
    Vector3d diff = v_p->estimate()-true_points[it->second];
    if (inliers.find(it->first)==inliers.end())
      continue;
    sum_diff2 += diff.dot(diff);
    ++point_num;
  }
  cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;
  cout << endl;
}
