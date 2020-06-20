#include <Eigen/Core>
#include <iostream>

#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

using namespace std;

/**
 * @brief 优化参数：a，b lambda 拟合曲线为：a*exp(-lambda*t) + b
 */
class VertexParams : public g2o::BaseVertex<3,Eigen::Vector3d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexParams()
        {
            
        }
        virtual bool read(std::istream& /*is*/)
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }
        virtual bool write(std::ostream& /*os*/) const
        {
            cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
            return false;
        }
        virtual void setToOriginImpl()
        {
            std::cout<<"not implemented!"<<std::endl;
        }
        virtual void oplusImpl(const double* update)
        {
            Eigen::Vector3d::ConstMapType v(update);
            _estimate += v;
        }
};

/**
 * @brief 一个点对应曲线中的一个测量值
 * 
 * */
class EdgePointOnCurve : public g2o::BaseUnaryEdge<1,Eigen::Vector2d,VertexParams>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgePointOnCurve(){}
    virtual bool read(std::istream& /*is*/)
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    virtual bool write(std::ostream& /*os*/) const
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    void computeError()
    {
        const VertexParams* params = static_cast<const VertexParams*>(vertex(0));
        const double& a = params->estimate()(0);
        const double& b = params->estimate()(1);
        const double& lambda = params->estimate()(2);
        double fval = a * exp(-lambda * measurement()(0)) + b;
        _error(0) = fval - measurement()(1);
    }

};

// main 函数入口
int main(int argc, char** argv)
{
// 参数变量
    int numPoints = 50;//采样点数量
    int maxIterations = 10;//迭代次数
    bool verbose = false;//是否输出详细调试信息
    string dumpFilename = "";//文件名称
    // 参数设置
    g2o::CommandArgs arg;
    arg.param("dump",dumpFilename,"","dump the points into a file");
    arg.param("numPoints",numPoints,50,"number of points sampled from the curve");
    arg.param("i",maxIterations,10,"perform n iterations");
    arg.param("v",verbose,false,"verbose output of the optimization process");

    arg.parseArgs(argc,argv);
    // 真实值
    double a = 2.;
    double b = 0.4;
    double lambda = 0.2;
    // 生成随机数据
    Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];
    for(int i=0;i<numPoints;i++)
    {
        double x = g2o::Sampler::uniformRand(0,10);
        double y = a*exp(-lambda*x) + b;
        // 添加高斯噪声
        y += g2o::Sampler::gaussRand(0,0.02);
        points[i].x() = x;
        points[i].y() = y;
    }
    // 求解过程

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic,Eigen::Dynamic> > MyBlockSolver;
    typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    // 设置求解器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

    optimizer.setAlgorithm(solver);
    // 构建优化问题
    // 1.添加待优化的节点
    VertexParams* params = new VertexParams();
    params->setId(0);
    params->setEstimate(Eigen::Vector3d(1,1,1));
    
    optimizer.addVertex(params);
    // 2.添加约束
    for(int i=0;i<numPoints;i++)
    {
        EdgePointOnCurve* e = new EdgePointOnCurve();
        e->setInformation(Eigen::Matrix<double,1,1>::Identity());
        e->setVertex(0,params);
        e->setMeasurement(points[i]);
        optimizer.addEdge(e);
    }
    // 执行优化
    optimizer.initializeOptimization();
    optimizer.setVerbose(verbose);
    optimizer.optimize(maxIterations);

    cout << "Target curve" << endl;
    cout << "a * exp(-lambda * x) + b" << endl;
    cout << "Iterative least squares solution" << endl;
    cout << "a      = " << params->estimate()(0) << endl;
    cout << "b      = " << params->estimate()(1) << endl;
    cout << "lambda = " << params->estimate()(2) << endl;
    cout << endl;
    delete[] points;
    return 0;
}

