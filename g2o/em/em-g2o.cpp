#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <cmath>

using namespace std;
using namespace Eigen;


//顶点，即待优化变量，目标值
class CurveFittingVertex: public g2o::BaseVertex<4,Vector4d> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex():BaseVertex<4,Vector4d>()
    {

    }
    
    virtual void setToOriginImpl()
    {
        _estimate << 0,0,0,0;
    }

    virtual void oplusImpl(const double *update_) //更新顶点
    {
        Eigen::Map<const Vector4d> up(update_);
        _estimate += up;
        // cout<<"eee" <<_estimate<<endl;
    }

    bool read(std::istream& is){}
    bool write(std::ostream& os) const{}


};

//边，描述顶点之间的关系
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge():g2o::BaseUnaryEdge<1,double,CurveFittingVertex>(){}
// 计算误差
    void computeError()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Vector4d abcd = v->estimate();
        double A = abcd[0],B = abcd[1],C = abcd[2],D = abcd[3];
        _error(0,0) = _measurement - (A*sin(B*_x)+C*cos(D*_x)); // 观测量减去估计量
        // cout << "cee "<<_error << endl;

    }
// 计算雅可比矩阵
    void linearizeOplus()
    {
        CurveFittingVertex *vi = static_cast<CurveFittingVertex *>(_vertices[0]);
        Vector4d abcd = vi->estimate();
        double A = abcd[0],B = abcd[1],C = abcd[2],D = abcd[3];
        // cout << " ddd" << endl;
        //误差项对待优化变量的Jacobian
        _jacobianOplusXi(0,0) = -sin(B*_x);
        _jacobianOplusXi(0,1) = -A*_x*cos(B*_x);
        _jacobianOplusXi(0,2) = -cos(D*_x);
        _jacobianOplusXi(0,3) = C*_x*sin(D*_x);
        
        
    }

    bool read(istream &is){}
    bool write(ostream &os) const {}

public:
    double _x;
};

int main(int argc, char**argv)
{
    // double a = 5.0,b = 1.0,c = 10.0,d = 2.0;
    // int N = 100;

    // double w_sigma = 2.0;

    // cv::RNG rng;

    // double abcd[4] = {0.0,0.0,0.0};

    // vector<double> x_data,y_data;

    // cout << "generate data" << endl;

    // for (int i = 0; i < N; i++)
    // {
    //     double x = rng.uniform(-10,10);
    //     double y = a*sin(b*x)+c*cos(d*x)+rng.gaussian(w_sigma);
    //     x_data.push_back(x);
    //     y_data.push_back(y);

    //     // cout << x_data[i] << " ," << y_data[i] << endl;

    // }

    // // 每个误差项优化变量维度为 4，误差值维度为1
    // typedef g2o::BlockSolver<g2o::BlockSolverTraits<4,1>> Block;
    // // 线性方程求解器： 稠密的增量方程
    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();

    // // 矩阵块求解器
    // Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
    
    // // 梯度下降方法
    // // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
    // g2o::OptimizationAlgorithmDogleg *solver = new g2o::OptimizationAlgorithmDogleg(std::unique_ptr<Block>(solver_ptr));
    
    // g2o::SparseOptimizer optimizer;
    // optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);

    // CurveFittingVertex *v = new CurveFittingVertex();
    // // 初始值
    // v->setEstimate(Eigen::Vector4d(1.6,1.4,6.2,1.7));
    // v->setId(0);
    // v->setFixed(false);
    // optimizer.addVertex(v);//添加顶点

    // for(int i=0;i< N;i++)
    // {
    //     CurveFittingEdge *edge = new CurveFittingEdge();
    //     edge->setId(i+1);
    //     edge->setVertex(0,v);//设置连接的顶点
    //     edge->setMeasurement(y_data[i]);

    //     //信息矩阵： 协方差矩阵之逆
    //     edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma));
    //     edge->_x = x_data[i];
    //     optimizer.addEdge(edge);

    // }

    // cout << "start optimization" << endl;

    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // optimizer.initializeOptimization();
    // optimizer.optimize(100);

    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << " time_used: " << time_used.count() << " seconds" << endl;

    // Eigen::Vector4d abcd_estimate = v->estimate();
    // cout << "estimated \n" << abcd_estimate << endl;

    // return 0;


    double a = 5.0, b = 1.0, c = 10.0, d = 2.0; // 真实参数值
    int N = 100;
    double w_sigma = 0.2;   // 噪声值Sigma
    cv::RNG rng;    // 随机数产生器OpenCV
    double abcd[4] = {0, 0, 0, 0};  // 参数的估计值abc

    vector<double> x_data, y_data;

    cout << "generate random data" << endl;

    for(int i = 0; i < N; i++)
    {
        //generate a random variable [-10 10]
        double x = rng.uniform(-10., 10.);
        double y = a * sin(b*x) + c * cos(d *x) + rng.gaussian(w_sigma);
        // double y = a * sin(b*x) + c * cos(d *x);
        x_data.push_back(x);
        y_data.push_back(y);

        // cout << x_data[i] << " , " << y_data[i] << endl;
    }

    // 构建图优化，先设定g2o
    // 矩阵块：每个误差项优化变量维度为4 ，误差值维度为1
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<4, 1> > Block;
    // 线性方程求解器：稠密的增量方程
    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();

    typedef g2o::LinearSolverDense<Block::PoseMatrixType> MyLinearSolver;
    // Block* solver_ptr = new Block(linearSolver);    // 矩阵块求解器

    // // 梯度下降方法，从GN, LM, DogLeg 中选
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    // 矩阵块求解器
    // Block* solver_ptr = new Block(std::make_unique<Block::LinearSolverType>(linearSolver));
    // g2o::OptimizationAlgorithmDogleg *solver = new g2o::OptimizationAlgorithmDogleg(std::unique_ptr<Block>(solver_ptr));
    g2o::SparseOptimizer optimizer;     // 图模型
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<Block>(g2o::make_unique<MyLinearSolver>()));
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(g2o::make_unique<Block>(g2o::make_unique<MyLinearSolver>()));
    
    optimizer.setAlgorithm( solver );   // 设置求解器
    optimizer.setVerbose(true);     // 打开调试输出

    // 往图中增加顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    // 设置优化初始估计值
    v->setEstimate( Eigen::Vector4d(1.6, 1.4, 6.2, 1.7));
    v->setId(0);
    // v->setFixed(false);
    optimizer.addVertex(v);

    // 往图中增加边
    for(int i = 0; i < N; i++)
    {
        CurveFittingEdge* edge = new CurveFittingEdge();
        edge->setId(i+1);
        edge->setVertex(0, v);      // 设置连接的顶点
        edge->setMeasurement( y_data[i] );      // 观测数值

        // 信息矩阵：协方差矩阵之逆
        edge->setInformation( Eigen::Matrix<double, 1, 1>::Identity() );

        edge->_x = x_data[i];

        optimizer.addEdge( edge );
    }

    // 执行优化
    cout << "strat optimization" << endl;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    optimizer.initializeOptimization();
    optimizer.optimize(500);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double> > (t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds." << endl;

    // 输出优化值
    Eigen::Vector4d abcd_estimate = v->estimate();
    cout << "estimated module: " <<  endl << abcd_estimate << endl;

    return 0;
    

}


